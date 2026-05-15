# Active LC Planner — Implementation Notes v0.1.1

## 環境

- Language: **C++17**
- Framework: **ROS2 Humble**
- SLAM backend: **RTAbMap**

---

## 設計哲學與文獻定位

### 核心主張（Ref1 §I）

**Active Loop Closure 是 exploration quality 的內生組成，而非探索結束後的 SLAM 補救動作。**

傳統 frontier-based 探索只追求覆蓋速度，不考慮 pose graph 漂移。SLAM 誤差累積後，occupancy map 形變可能封堵窄道、阻斷探索。本 planner 在探索過程中持續評估 loop closure 的報酬，在必要時主動介入，使 SLAM 品質成為探索決策的一部分。

### 從兩篇文獻保留的核心

<table>
  <thead>
    <tr>
      <th></th>
      <th>Ref1（Yin et al.）</th>
      <th>Ref2（Kim &amp; Eustice）</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>保留</th>
      <td>
        <table>
          <tr><td>Reward 結構骨幹（旅行成本 + 機率加權 ΔU）</td></tr>
          <tr><td>Distance-Based uncertainty surrogate</td></tr>
          <tr><td>BNB target selection</td></tr>
          <tr><td>Adaptive triggering</td></tr>
        </table>
      </td>
      <td>
        <table>
          <tr><td>Dual saliency（S_L, S_G）做 candidate 篩選</td></tr>
          <tr><td>DBSCAN 聚類</td></tr>
          <tr><td>Coverage term 的設計概念</td></tr>
        </table>
      </td>
    </tr>
    <tr>
      <th>不照搬</th>
      <td>
        <table>
          <tr><td>Cluster P_LC 獨立性假設：系統性高估，僅作 ranking signal</td></tr>
          <tr><td>ΔU 只描述 pairwise potential，無法反映全域幾何結構</td></tr>
        </table>
      </td>
      <td>
        <table>
          <tr><td>P_L 的 empirical prior：場景特化，不可遷移</td></tr>
          <tr><td>uncertainty 與 coverage 的線性加權：兩者量綱不同</td></tr>
          <tr><td>Candidate 生成限定於 nominal survey pattern</td></tr>
        </table>
      </td>
    </tr>
  </tbody>
</table>

---

## v0.1.1 與初始計畫的重要差異

以下列出實作中與原始設計文件有明顯出入的地方，優先閱讀。

### 1. 架構縮減：非 4-option 決策迴圈

**初始計畫**：`SLAMGraphPlanner` 每個 cycle 評估四個 motion option（`EXPAND_ANCHOR`、`REVISIT_ALC`、`CLOSE_GAP`、`FRONTIER_EXPAND`），選 utility 最高者執行。

**實際實作**：`SLAMGraphPlanner` 只管 ALC 決策，狀態只有三個：

```
EVALUATING → NAVIGATING_TO_ALC → ROTATING → EVALUATING
```

`PlannerAction` 只有 `NO_ACTION` 和 `REVISIT_ALC`。Frontier expansion 完全在本 planner 外部處理（由 `planner_node` 層負責）。`EXPAND_ANCHOR` 和 `CLOSE_GAP` 尚未實作。

### 2. 無狀態 Planning Cycle

`runPlanningCycle()` 是一個無狀態的 free function。每次呼叫會建立一個短暫的 `SLAMGraphPlanner` 實例，用完即丟。**狀態機的持久性由 `planner_node` 層自行管理**，不在 `planning_cycle` 內。

### 3. 資料結構拆分

`Keyframe` 只保留幾何與 BoW 資料，saliency 資料移至獨立的 `KeyframeSaliency`：

```cpp
struct Keyframe {
    int     node_id = -1;
    Pose6f  pose;
    std::vector<int32_t> word_ids;   // BoW word ids（用於計算 S_L, S_G）
    // viewing_dir 改為即時計算：pose.forward() = pose.orientation * (1,0,0)
};

struct KeyframeSaliency {
    float saliency_local  = 0.0f;
    float saliency_global = 0.0f;
    float plc_intrinsic   = 0.0f;
    bool  is_lighthouse   = false;
};

struct SaliencyState {
    std::vector<KeyframeSaliency> keyframes;
    float plc_calibration = 1.0f;    // 動態校準因子，見下方說明
};
```

`GraphEdge` 加入 `variance` 欄位，支援 variance-based ΔU（見 §2-B）：

```cpp
struct GraphEdge {
    int   to       = -1;
    float dist     = 0.0f;
    float variance = 0.0f;
};
```

### 4. P_LC 上界的計算方式

**計畫描述**（S_L_max / S_G_max 取 cluster 最大值）：
```
P_LC_ub = tanh(cv_L × S_L_max) × tanh(cv_G × S_G_max) × exp(−l_minus_ub²/cl²)
```

**實際實作**（各 keyframe 的 `plc_intrinsic` 乘以 exp factor 後加總，再 clamp）：
```cpp
float plc_sum = 0.f;
for (int kf_ix : candidate.keyframe_ixs) {
    plc_sum += plc_intrinsic(kf_ix) * exp(−l_minus_ub²/cl²);
}
candidate.P_lc_ub = std::min(1.0f, plc_sum);
```

此公式同時近似 P_LC_ub 並與 `fillReward` 的 cluster P_LC 計算風格一致，但**不嚴格是 Ref1 描述的 upper bound**，僅作 ranking signal 使用。

### 5. PLC 動態校準（計畫未提及）

`SaliencyEvaluator` 追蹤 loop closure 嘗試成功率，算出 `plc_calibration` 因子：

```
plc_calibration = clamp(observed_rate / prior_rate, 0.5, 2.0)
prior_rate = 0.5
```

所有 `plc_intrinsic` 在 `RewardEvaluator` 中會乘以此因子，再 clamp 到 [0,1]。這讓 P_LC 估計隨著實際場景的 loop closure 成功率動態調整。

### 6. 兩種 ΔU 計算模式（`use_variance_uncertainty`）

`Params` 新增：

```cpp
bool use_variance_uncertainty = false;
```

- `false`（預設）：distance-based ΔU = `graph_dist − map_dist`，upper bound = `graph_dist − euclidean_dist`
- `true`：variance-based ΔU = `graph_dist_var`（沿 Dijkstra 路徑累加邊的 `variance`），upper bound 同樣用 `graph_dist_var`

### 7. `selectRepresentative` 無 navigability check

**計畫描述**中 `selectRepresentative` 包含兩個必要條件：
1. viewing-direction compatibility（approach.dot(kf_fwd) ≥ 0）
2. `occupancyMap_.isReachable(kf.pose.position())`

**實際實作** 只做 viewing-direction check，沒有 navigability check。無法到達的候選點會在 BNB 中呼叫 A* 時因 `map_dist = inf` 被跳過。

### 8. BNB 改為排序後 break

**計畫描述**的 pseudocode 每輪用 `max_element` 找最高 `reward_ub` 再計算。

**實際實作** 在 `select()` 開頭先對所有 candidate 按 `reward_ub` 降序排列，然後線性掃描，遇到 `candidate.reward_ub ≤ best.reward` 就直接 break：

```cpp
std::sort(candidates, by reward_ub descending);
for (auto& c : candidates) {
    if (best && c.reward_ub <= best->reward) break;
    c.map_dist = path_planner_.computeDist(...);
    evaluator_.fillReward(c, saliency_state);
    if (!best || c.reward > best->reward) best = c;
}
```

行為等價，實作更簡潔。

### 9. Lighthouse 建立邏輯未實作

`KeyframeSaliency::is_lighthouse` 欄位存在，但主動觸發 lighthouse（偵測 `S_L > cs_lighthouse` → 原地旋轉 → 登記）的邏輯尚未實作。

---

## 系統架構（v0.1.1）

```
┌──────────────────────────────────────────────────────────┐
│                   planner_node（ROS2 Node）               │
│                                                          │
│  訂閱 mapData, info, /map                                │
│  維護 GraphState、SaliencyState、SLAMGraphPlanner 狀態    │
│  外部 frontier expansion 邏輯在此層                      │
│                                                          │
│  ┌──────────────────────────────────────────────────┐    │
│  │  runPlanningCycle()  ← stateless free function   │    │
│  │                                                  │    │
│  │  CandidateBuilder → filter → DBSCAN → rep select │    │
│  │  RewardEvaluator  → fillRewardUB（cheap）         │    │
│  │  BNBSelector      → sort → A* loop → best τ*     │    │
│  │  SLAMGraphPlanner → shouldTriggerALC？            │    │
│  │                                                  │    │
│  │  returns: PlanningCycleResult                    │    │
│  │    .action ∈ {NO_ACTION, REVISIT_ALC}            │    │
│  │    .best_candidate                               │    │
│  │    .triggered_candidate                          │    │
│  └──────────────────────────────────────────────────┘    │
│                                                          │
│  若 action == REVISIT_ALC：                              │
│    publish navigate_to_pose(τ*.rep_pose)                 │
│    on success → rotate_in_place(360°) → reset timer      │
└──────────────────────────────────────────────────────────┘
```

---

## 符號定義（對應 Ref1 §III）

| 符號             | 意義                                                      |
| ---------------- | --------------------------------------------------------- |
| `G = (V, E)`     | Pose graph；V 為 keyframe 節點，E 為邊                    |
| `v_r`            | 機器人當前節點（`graph.robot_ix`）                        |
| `ix`             | 節點的內部索引；對應外部 `node_id` 的映射由 `ix_to_node` 維護 |
| `l(p_i, p_j, M)` | Occupancy map 上 A* 最短路徑距離                          |
| `l(p_i, p_j, G)` | Pose graph 上沿邊累加的最短路徑距離                       |
| `d(p_i, p_j)`    | Euclidean 距離                                            |
| `S_L(v)`         | Local saliency：影像紋理豐富度，∈ [0,1]                   |
| `S_G(v)`         | Global saliency：影像在地圖中的稀有度，∈ [0,1]             |

---

## 資料結構

```cpp
struct Keyframe {
    int     node_id = -1;
    Pose6f  pose;
    std::vector<int32_t> word_ids;   // BoW word ids，用於計算 saliency
    // viewing_dir 不存儲；需要時用 pose.forward() 即時計算
};

struct KeyframeSaliency {
    float saliency_local  = 0.0f;   // S_L: 正規化 word count
    float saliency_global = 0.0f;   // S_G: word frequency 倒置
    float plc_intrinsic   = 0.0f;   // tanh(cv_L × S_L) × tanh(cv_G × S_G)
    bool  is_lighthouse   = false;  // 登記為 lighthouse（建立邏輯待實作）
};

struct GraphEdge {
    int   to       = -1;
    float dist     = 0.0f;
    float variance = 0.0f;   // 用於 variance-based ΔU（opt-in）
};

struct GraphState {
    std::vector<Keyframe>            keyframes;
    std::vector<std::vector<GraphEdge>> adj;
    std::unordered_map<int, int>     node_to_ix;
    std::vector<int>                 ix_to_node;
    int robot_ix = -1;
    std::uint64_t version = 0;
};

struct SaliencyState {
    std::vector<KeyframeSaliency> keyframes;
    float plc_calibration = 1.0f;   // 動態校準因子
};

struct ALCCandidate {
    int   tau_ix         = -1;      // representative 的內部索引
    Pose6f rep_pose;
    std::vector<int> keyframe_ixs;  // cluster 成員的內部索引
    float euclidean_dist = 0.0f;
    float graph_dist     = 0.0f;
    float graph_dist_var = 0.0f;    // variance-mode 的 Dijkstra 結果
    float map_dist       = 0.0f;
    float P_lc           = 0.0f;
    float delta_U        = 0.0f;
    float delta_U_ub     = 0.0f;
    float P_lc_ub        = 0.0f;
    float reward         = 0.0f;
    float reward_ub      = 0.0f;
    bool  is_lighthouse  = false;
};
```

### 超參數（`Params`）

| 參數                      | 意義                                  | 初始值  |
| ------------------------- | ------------------------------------- | ------- |
| `cv_L`                    | S_L tanh 斜率                         | 3.0     |
| `cv_G`                    | S_G tanh 斜率                         | 3.0     |
| `cl`                      | P_LC distance decay 尺度              | 10.0 m  |
| `ct`                      | 旅行成本權重                          | 0.1     |
| `cE`                      | 最大候選距離（euclidean）              | 15.0 m  |
| `cG`                      | 最小 graph distance（uncertainty 門檻）| 3.0 m   |
| `cs`                      | 最小 local saliency（篩選門檻）        | 0.3     |
| `cs_lighthouse`           | Lighthouse 觸發 saliency（未使用）    | 0.6     |
| `d_min_lighthouse`        | Lighthouse 最小間距（未使用）         | 5.0 m   |
| `eps_dbscan`              | DBSCAN 鄰域半徑                       | 1.5 m   |
| `min_pts`                 | DBSCAN 最小點數                       | 2       |
| `theta_max`               | ALC trigger 初始 threshold            | 0.5     |
| `lambda_decay`            | Threshold 時間衰減率                  | 0.01 /s |
| `alpha_cov`               | Coverage 調節 threshold 的斜率        | 0.3     |
| `use_variance_uncertainty`| 切換 ΔU 計算模式                      | false   |

---

## 模組一：Saliency 計算

### Local Saliency（S_L）

- 衡量影像紋理豐富程度
- Proxy：`word_ids.size()`（keyframe 的 BoW 命中詞數）正規化至 `[0,1]`，分母取歷史最大值

```cpp
S_L = word_count / max_words_recognized_seen_so_far
```

### Global Saliency（S_G）

- 衡量影像在地圖中的稀有度
- Proxy：每個 word 在所有 keyframe 中的平均出現頻率倒置，再用 log 壓縮並正規化

```cpp
raw_sg = mean(1 / freq_of_word_in_graph)   // 對 keyframe 的 unique words 平均
S_G = log(raw_sg) / log(total_nodes)       // 正規化到 [0,1]
```

### plc_intrinsic

```
P_LC_intrinsic(v) = tanh(cv_L × S_L(v)) × tanh(cv_G × S_G(v))
```

用於 representative 選取與 P_LC 計算。

### plc_calibration（動態校準）

`SaliencyEvaluator` 追蹤每次 loop closure 嘗試的結果：

```cpp
plc_calibration = clamp(observed_rate / 0.5, 0.5, 2.0)
```

所有 `plc_intrinsic` 在 reward 計算前乘以此因子並 clamp 到 [0,1]。環境 loop closure 成功率高時放大估計、低時縮小。

---

## 模組二：Uncertainty Metrics

### 2-A. Distance-Based ΔU（預設，`use_variance_uncertainty = false`）

```
ΔU(v)   = l(v_r, v, G) − l(v_r, v, M)   // 精確值（需 A*）
ΔU_ub   = l(v_r, v, G) − d(v_r, v)      // 上界（cheap）
```

### 2-B. Variance-Based ΔU（`use_variance_uncertainty = true`）

以邊的 `variance` 加權跑 Dijkstra，`graph_dist_var` 為累積 variance：

```
ΔU(v) = ΔU_ub = graph_dist_var   // 不需 A*，upper bound = exact value
```

此模式不需要 A* 就能得到精確值，但語意是 pose uncertainty 的 variance proxy，而非 path length 差。

---

## 模組三：ALC Candidate Construction

### Keyframe 篩選（Ref1 Alg.1 + 實際實作）

```cpp
bool shouldInclude(ix, robot_pose, dist_map) {
    return euclidean(robot_pose, keyframes[ix].pose) <= cE   // 距離上限
        && saliency_state.keyframes[ix].saliency_local >= cs // saliency 下限
        && dist_map[ix] >= cG;                               // graph dist 下限
}
```

### DBSCAN 聚類

標準 DBSCAN，以 euclidean distance 為鄰域判斷：

```cpp
clusters = dbscan(filtered_ixs, eps_dbscan, min_pts)
```

### Representative 選取

```cpp
int selectRepresentative(cluster_ixs, robot_pose) {
    // 第一輪：通過 viewing-direction check 的節點中，選 plc_intrinsic 最高者
    for (ix in cluster_ixs) {
        approach = (kf.pose.position − robot_pose.position).normalized()
        if approach.dot(kf.pose.forward()) < 0.0f: skip   // 背面剔除
        score = saliency_state.keyframes[ix].plc_intrinsic
        track best
    }
    // Fallback：若全部不通過 viewing-direction，退回整個 cluster 的 plc_intrinsic 最高者
    if no valid: return argmax(plc_intrinsic, cluster_ixs)
}
```

**注意**：沒有 navigability check。無法到達的候選點在 BNB 的 A* 步驟中會因 `map_dist = inf` 而被跳過（`fillReward` 前提是 `isfinite(map_dist)`）。

---

## 模組四：Loop Closure Probability（P_LC）

### Keyframe-level P_LC

```
plc_i = clamp(plc_intrinsic_i × plc_calibration, 0, 1)
       × exp(−l_minus² / cl²)

l_minus = graph_dist + map_dist
```

### Cluster P_LC（Ref1 Eq.9，精確值）

```cpp
float prob_all_fail = 1.0f;
for (ix in keyframe_ixs) {
    prob_all_fail *= (1.0f - plc_i);
}
P_lc = 1.0f - prob_all_fail;
```

### Cluster P_LC 上界（BNB 使用）

```cpp
float plc_sum = 0.0f;
for (ix in keyframe_ixs) {
    plc_sum += plc_intrinsic_i * plc_calibration * exp(−l_minus_ub²/cl²);
}
P_lc_ub = min(1.0f, plc_sum);   // 加總後 clamp
```

`l_minus_ub = graph_dist + euclidean_dist`（以 euclidean 替換 map_dist，因為 euclidean ≤ map_dist）。此公式**不嚴格是 upper bound**，但在 candidate 數目較少時作 ranking signal 足夠。

---

## 模組五：Reward Function（Ref1 Eq.10）

```
R(τ) = −ct × map_dist  +  P_lc(τ) × ΔU(τ)
```

```
R_ub(τ) = −ct × euclidean_dist  +  P_lc_ub(τ) × ΔU_ub(τ)
```

兩個版本都在 `RewardEvaluator` 中計算：

- `fillRewardUB`：只需 `graph_dist` 和 `euclidean_dist`（不跑 A*）
- `fillReward`：需要 `map_dist`（由 BNB 在呼叫 A* 後填入）

---

## 模組六：BNB Target Selection

```cpp
std::optional<ALCCandidate> BNBSelector::select(candidates, graph, saliency, map)
{
    // 先對所有候選點計算 R_ub（已在 planning_cycle 中完成）
    std::sort(candidates, by reward_ub descending);

    std::optional<ALCCandidate> best;
    for (auto& c : candidates) {
        if (best && c.reward_ub <= best->reward) break;   // 剩餘候選無法超越 best

        c.map_dist = path_planner_.computeDist(robot_pos, c.rep_pose.position, map, ...);
        if (!isfinite(c.map_dist)) continue;              // 不可到達

        evaluator_.fillReward(c, saliency_state);
        if (!best || c.reward > best->reward) best = c;
    }
    return best;
}
```

---

## 模組七：Trigger Policy

```cpp
bool TriggerPolicy::shouldTriggerALC(reward, elapsed_seconds, coverage_ratio) {
    double threshold = theta_max × exp(−lambda_decay × elapsed_seconds);
    threshold *= (1.0 + alpha_cov × (1.0 − coverage_ratio));
    return reward > threshold;
}
```

- 距上次 ALC 越久（elapsed 越大），threshold 越低，越容易觸發
- Coverage 越低（frontier 還多），threshold 適度提升

---

## 模組八：SLAMGraphPlanner 狀態機

`SLAMGraphPlanner` 維護 ALC 執行階段：

```
EVALUATING
    │  onEvaluationComplete(best, elapsed, coverage)
    │  → shouldTriggerALC(best.reward, ...) == true
    ▼
NAVIGATING_TO_ALC
    │  onNavigationResult(success=true)
    ▼
ROTATING
    │  onRotationComplete()
    ▼
EVALUATING

─── 失敗路徑 ───
NAVIGATING_TO_ALC
    │  onNavigationResult(success=false)
    ▼
EVALUATING
```

**關鍵**：`runPlanningCycle()` 每次都建立新的 `SLAMGraphPlanner` 實例（no persistent state）。狀態的跨 cycle 持久性由 `planner_node` 自行維護；`planning_cycle` 只負責計算 best candidate 與 trigger 判斷。

---

## RTAbMap 整合

### 訂閱

| Topic                | 用途                                                              |
| -------------------- | ----------------------------------------------------------------- |
| `/rtabmap/mapData`   | Pose graph（nodes, edges, links）；`word_ids` 從 node data 取得   |
| `/rtabmap/info`      | Loop closure 事件（`loopClosureId > 0`）；`wordsRecognized` 計數  |
| `/map`               | Occupancy map，A* 使用                                            |

### 發布

| Topic / Action          | 用途                  |
| ----------------------- | --------------------- |
| Nav2 `navigate_to_pose` | 導航至 τ*             |
| 自訂 `rotate_in_place`  | 原地 360° 旋轉        |
| `/alc_planner/status`   | Planner 狀態（debug） |

### Saliency Proxy

| Saliency        | Proxy                                                                           |
| --------------- | ------------------------------------------------------------------------------- |
| `S_L`（local）  | `word_ids.size()` 正規化（歷史最大值為分母）                                    |
| `S_G`（global） | BoW word frequency 倒置：`mean(1/freq)` → log-normalize by `log(total_nodes)` |

---

## 實作進度

```
Phase 1 — Skeleton                        ✅ 完成
  ✅ GraphState, SaliencyState, ALCCandidate 型別定義
  ✅ Dijkstra（distance + variance 兩種模式）
  ✅ Saliency proxy（S_L + S_G + plc_intrinsic + plc_calibration）

Phase 2 — Candidate Construction          ✅ 完成
  ✅ DBSCAN 聚類
  ✅ Representative 選取（viewing-direction + plc_intrinsic）
  ✅ Lighthouse flag（is_lighthouse）
  □ Lighthouse 主動觸發與登記（未實作）

Phase 3 — Metrics & P_LC                 ✅ 完成
  ✅ Distance-Based ΔU（Ref1 Eq.8）
  ✅ Variance-Based ΔU（opt-in）
  ✅ P_LC heuristic（Ref1 Eq.9）
  ✅ R(τ) 與 R_ub(τ)

Phase 4 — BNB Target Selection           ✅ 完成
  ✅ A* map distance（PathPlanner）
  ✅ BNB 選取 τ*（sort + break）

Phase 5 — State Machine & Triggering     ✅ 完成（ALC 路徑）
  ✅ Adaptive threshold（TriggerPolicy）
  ✅ EVALUATING / NAVIGATING_TO_ALC / ROTATING 狀態
  □ EXPAND_ANCHOR / CLOSE_GAP / FRONTIER_EXPAND（未實作，在 planner_node 層）

Phase 6 — 擴充                            □ 未開始
  □ Saliency-weighted A*（Ref2）
  □ Neighbor D-optimality（若 RTAbMap 能提供 covariance）
  □ P_LC empirical model
  □ Coverage ratio 精化
```

---

## 論文方法對照

| 來源 | 章節          | 方法                             | 本計畫模組     | 狀態     |
| ---- | ------------- | -------------------------------- | -------------- | -------- |
| Ref1 | §IV-B, Eq.8   | Distance-Based ΔU                | 模組二 2-A     | ✅       |
| Ref1 | §IV-A, Eq.5-6 | Neighbor D-optimality            | 模組二 2-B     | □ Phase 6 |
| Ref1 | §V-A, Alg.1   | ALC Candidate Construction       | 模組三         | ✅       |
| Ref1 | §V-B, Eq.9    | P_LC heuristic                   | 模組四         | ✅       |
| Ref1 | §V-C, Eq.10   | Reward function                  | 模組五         | ✅       |
| Ref1 | §V-D.1        | Reward upper bound R_ub          | 模組五         | ✅ (近似) |
| Ref1 | §V-D.2, Alg.2 | Branch and Bound selection       | 模組六         | ✅       |
| Ref1 | §VI-A         | Adaptive triggering threshold    | 模組七         | ✅       |
| Ref1 | §V-A          | Lighthouse 建立                  | —              | □ 部分   |
| Ref2 | §3            | Dual saliency（S_L + S_G）       | 模組一         | ✅       |
| Ref2 | §4.1          | DBSCAN clustering + rep selection | 模組三        | ✅       |
| Ref2 | §4.2          | Saliency-weighted A*             | —              | □ Phase 6 |
| Ref2 | §4.3.3-4      | Coverage term（trigger 調節）    | 模組七         | ✅       |

---

## References

```
[Ref1] He Yin, Jong Jin Park, Marcelino Almeida, Martin Labrie, Jim Zamiska, Richard Kim.
       "Probabilistic Active Loop Closure for Autonomous Exploration." 2024.
       https://www.amazon.science/publications/probabilistic-active-loop-closure-for-autonomous-exploration

[Ref2] Ayoung Kim, Ryan M. Eustice.
       "Active Visual SLAM for Robotic Area Coverage: Theory and Experiment."
       IJRR / ICRA 2013.
```
