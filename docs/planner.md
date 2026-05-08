# Active LC Planner — Implementation Plan v0.1

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
          <tr><td>Decision formulation 骨架（revisit vs explore 的明確比較）</td></tr>
          <tr><td>Dual saliency（S_L, S_G）做 candidate 篩選</td></tr>
          <tr><td>DBSCAN 聚類</td></tr>
          <tr><td>Saliency-weighted A*</td></tr>
          <tr><td>Coverage term 的設計概念</td></tr>
        </table>
      </td>
    </tr>
    <tr>
      <th>不照搬</th>
      <td>
        <table>
          <tr>
            <td>
              Cluster P_LC 獨立性假設：鄰近 keyframe 的回環事件正相關，獨立性乘法公式會<strong>高估</strong> P_LC；正相關時整批失敗機率高於獨立假設，因此 cluster 成功機率低於公式預測。
            </td>
          </tr>
          <tr><td>ΔU 只描述 pairwise potential，無法反映全域幾何結構。</td></tr>
        </table>
      </td>
      <td>
        <table>
          <tr><td>P_L 的 empirical prior：場景特化，不可遷移。</td></tr>
          <tr><td>uncertainty 與 coverage 的線性加權：兩者量綱不同，交換性未被論證。</td></tr>
          <tr><td>Candidate 生成限定於 nominal survey pattern。</td></tr>
        </table>
      </td>
    </tr>
  </tbody>
</table>


### 設計立場

1. **Reward 結構用 Ref1**：R(τ) 只評估「去哪個 ALC 候選點最好」，不混入 coverage。Coverage 只影響「現在值不值得暫停去做 ALC」的 trigger 決策（兩個獨立的決策層）。
2. **決策主體是 SLAM graph planner**，frontier expansion 是它的選項之一，不是預設主迴圈。每個 cycle 評估四個 motion option，選報酬最高者執行。
3. **Candidate 生成混合兩者**：DBSCAN（Ref2）替代 Ref1 的固定半徑聚類，但候選點篩選邏輯仍用 Ref1 的 graph distance / view score 門檻
4. **S_L 和 S_G 統一進同一個 P_LC 決策模型**：`P_LC_intrinsic = tanh(cv_L × S_L) × tanh(cv_G × S_G)`，不分兩層各自為政。Representative 選取也用 P_LC_intrinsic 排序，讓 appearance quality 的兩個維度在同一個 criterion 下競爭，而不是 S_L 管 P_LC、S_G 管 candidate selection。
5. **Representative 選取加入 viewing-direction compatibility**（必要條件，非可選）：approach direction 與 keyframe viewing direction 不相容（夾角 > 90°）時，loop closure appearance 完全不同，直接剔除。
6. **BNB 保留 Ref1**；saliency-weighted A*（Ref2）在後期優化路徑規劃時引入

---

## 系統架構

```
┌──────────────────────────────────────────────────────────────────────┐
│                        SLAMGraphPlanner（決策主體）                   │
│                                                                      │
│  每個 cycle 評估四個 motion option，選報酬最高者執行：                  │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │  MotionOptionEvaluator                                         │  │
│  │                                                                │  │
│  │  EXPAND_ANCHOR   → anchor 鄰近的 frontier，局部 uncertainty 低  │  │
│  │  REVISIT_ALC     → R(τ*)：去 ALC target，預期 SLAM improvement │  │
│  │  CLOSE_GAP       → 鎖定 topological gap，補 loop closure        │  │
│  │  FRONTIER_EXPAND → U_frontier：information gain，覆蓋新區域     │  │
│  └──────────────────────────────┬─────────────────────────────────┘  │
│                                 │                                    │
│           ┌─────────────────────┼──────────────────┐                 │
│           ▼                     ▼                  ▼                 │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐        │
│  │ Candidate Builder│  │ Reward Evaluator │  │ BNB Selector │        │
│  │ DBSCAN + filters │  │ ΔU + P_LC only   │  │ (Ref1 Alg.2) │       │
│  │ (Ref1 + Ref2)    │  │ (Ref1 Eq.10)     │  │              │        │
│  └──────────────────┘  └──────────────────┘  └──────────────┘        │
└──────────────────────────────────────────────────────────────────────┘
          │                                               │
          ▼                                               ▼
   RTAbMap mapData                               Nav2 navigate_to_pose
   (keyframes, edges, links)                    + rotate_in_place
```

**決策主體是 SLAM graph planner**，frontier expansion 是它的 motion option 之一，不是預設主迴圈。Coverage 狀態只影響各 option 的相對優先級（例如：coverage 已充分時降低 FRONTIER_EXPAND 的 utility），不進入 ALC reward 的計算。

---

## 符號定義（對應 Ref1 §III）

| 符號             | 意義                                                      |
| ---------------- | --------------------------------------------------------- |
| `G = (V, E)`     | Pose graph；V 為 keyframe 節點，E 為邊                    |
| `v_r`            | 機器人當前節點                                            |
| `p_v`            | 節點 v 的 6-DoF pose（SE3）                               |
| `l(p_i, p_j, M)` | Occupancy map 上 A* 最短路徑距離                          |
| `l(p_i, p_j, G)` | Pose graph 上沿邊累加的最短路徑距離                       |
| `d(p_i, p_j)`    | Euclidean 距離                                            |
| `S_L(v)`         | Local saliency：影像紋理豐富度，∈ [0,1]（Ref2 §3）        |
| `S_G(v)`         | Global saliency：影像在地圖中的稀有度，∈ [0,1]（Ref2 §3） |

---

## 資料結構

```cpp
struct Keyframe {
    int     node_id;
    Pose6f  pose;            // RTAbMap node pose（含 orientation）
    Vec3f   viewing_dir;     // 觀測方向：pose.rotation * (1,0,0)，記錄 keyframe 建立時的朝向
    float   saliency_local;  // S_L: texture richness（wordsRecognized 正規化）
    float   saliency_global; // S_G: scene rarity（BoW word frequency 倒置）
    float   plc_intrinsic;   // tanh(cv_L × S_L) × tanh(cv_G × S_G)，不含距離項
    bool    is_lighthouse;
};

struct ALCCandidate {
    int                  tau_id;
    Pose6f               rep_pose;        // 代表性 pose p_τ
    std::vector<int>     keyframe_ids;    // 鄰域 keyframe V(τ)
    float                map_dist;        // l(p_r, p_τ, M) — A* 計算
    float                graph_dist;      // l(p_r, p_τ, G) — Dijkstra
    float                euclidean_dist;  // d(p_r, p_τ) — 用於 R_ub
    float                P_lc;           // Eq.9（Ref1）：loop closure 機率（高估，見 P_LC 模組說明）
    float                delta_U;        // Eq.8（Ref1）：uncertainty reduction（需 A*）
    float                delta_U_ub;     // ΔU upper bound = graph_dist − euclidean_dist（cheap）
    float                P_lc_ub;        // P_LC upper bound（以 euclidean_dist 替換 map_dist 計算）
    float                reward;         // R(τ)：完整 reward（需 A*）
    float                reward_ub;      // R_ub(τ)：upper bound（全 cheap，BNB 使用）
    bool                 is_lighthouse;
};
```

---

## 模組一：Saliency 計算（整合 Ref2 §3）

Ref1 使用單一 view score（feature count / NMS region）。Ref2 提出 **雙層 saliency**，更能區分哪些位置對 loop closure 真正有價值：

### Local Saliency（S_L）

- 衡量**影像內**紋理豐富程度（intra-image texture richness）
- 與 pairwise keyframe registration 成功率強相關
- 實作：RTAbMap `/rtabmap/info` 中的 `wordsRecognized`（當前幀命中 BoW 詞典的詞數）正規化（見 Saliency Proxy 一節）

### Global Saliency（S_G）

- 衡量影像在**整張地圖**中的稀有度（inter-image rarity）
- 高 S_G 代表場景在地圖中出現少 → revisit 時容易找到 unique match
- 實作：Bag-of-Words 詞頻倒置（TF-IDF 概念）；RTAbMap 有 BoW 詞典，可計算 word frequency

### 代表點選擇

在同一 cluster 中選 representative，需同時考慮三個條件（排序依重要性）：

1. **Viewing-direction compatibility**（必要條件）：機器人從當前位置接近此 keyframe 的方向，必須與 keyframe 當初記錄時的觀測方向相容。RTAbMap 的 loop closure 是 appearance-based，point-of-view 相差過大時 visual appearance 完全不同，loop closure 必然失敗。
2. **P_LC_intrinsic（= tanh(cv_L × S_L) × tanh(cv_G × S_G)）最大**：appearance quality 最高的節點（S_L 和 S_G 統一在同一個 ranking criterion 中）。
3. **Pose uncertainty 最低**（同 P_LC_intrinsic 相近時的 tie-breaker）。

```cpp
Keyframe selectRepresentative(const Cluster& cluster, const Pose6f& robot_pose) {
    const float cos_compat_threshold = 0.0f;  // 允許最大 90° 夾角

    float best_score = -1.f;
    const Keyframe* best = nullptr;

    for (const Keyframe& kf : cluster.keyframes) {
        // ── 1. Viewing-direction compatibility ──────────────────────────
        Vec3f approach = (kf.pose.position() - robot_pose.position()).normalized();
        Vec3f kf_fwd   = kf.pose.rotation() * Vec3f(1.f, 0.f, 0.f);  // 原始觀測方向
        if (approach.dot(kf_fwd) < cos_compat_threshold) continue;    // 背面，剔除

        // ── 2. Navigability check ────────────────────────────────────────
        if (!occupancyMap_.isReachable(kf.pose.position())) continue;

        // ── 3. P_LC_intrinsic 作為主要排序 ──────────────────────────────
        float score = std::tanh(cv_L_ * kf.saliency_local)
                    * std::tanh(cv_G_ * kf.saliency_global);

        if (score > best_score) { best_score = score; best = &kf; }
    }
    // fallback：若所有 keyframe 均不通過 view-direction filter，
    // 退回 cluster 中 P_LC_intrinsic 最高者（不保證 loop closure 成功，但保留候選點）
    return best ? *best : fallbackByIntrinsic(cluster);
}
```

**為何 navigability 是必要而不充分（Problem 8 回應）**：即使機器人能到達 representative 的 XY 位置，若接近方向（approach direction）與 keyframe 的 viewing direction 相差超過 ~90°，RTAbMap 看到的是截然不同的 visual scene，loop closure 不會觸發。navigability 只確保物理上能抵達，不確保 appearance 相容。兩個條件都需要。

---

## 模組二：Uncertainty Metrics（對應 Ref1 §IV）

### 2-A. Distance-Based Metric（優先實作，Ref1 Eq.8）

Pose graph 最短路徑距離 `l(v_r, v, G)` 是兩節點相對 uncertainty 的上界。加入 loop closure 邊後，graph distance 降至 map distance：

```
ΔU(p_v) = l(p_r, p_v, G) − l(p_r, p_v, M)
```

- `l(p_r, p_v, G)`：Dijkstra on RTAbMap pose graph edges
- `l(p_r, p_v, M)`：Nav2 `compute_path` service 或自行跑 A*
- ΔU ≤ 0 時（已有 loop closure）：該候選點跳過

**設計注意（pairwise 限制）**：此 surrogate 只描述 pairwise closure potential，無法反映整張圖的全域幾何結構。v0.1 以此為主，後續考慮加入 global metric proxy。

**模組四的 l_minus 說明（surrogate 免責）**：

Ref1 原始公式為 `l_minus = l(p_r, p_v, G) + l(p_r, p'_r, M)`，其中 `l(p_r, p'_r, M)` 是機器人**行走至 v 途中新累積的 odometry 距離**（p'_r 是抵達後的位置）。

本計畫寫成 `l_minus = graph_dist + map_dist` 是我們自己的 surrogate 解讀：假設機器人能精確抵達 v（`p'_r ≈ p_v`），則行走距離 ≈ map distance。**這不是 Ref1 公式的精確含義**，是我們在 RTAbMap 環境下的近似。在機器人需要大幅繞行（map_dist >> euclidean_dist）時，此近似可能高估 l_minus，低估 P_LC。

### 2-B. Neighbor D-optimality（後續擴充，Ref1 §IV-A）

```
Σ⁻¹_ij+ = Σ⁻¹_prior + Σ⁻¹_ij        (Ref1 Eq.5)
ΔU(p_v) = det(Σ_prior) / det(Σ̂_ij+)  (Ref1 Eq.6)
```

複雜度 O(1)（6×6 矩陣）。需要 RTAbMap 提供 covariance；RTAbMap 的 `mapGraph` service 可能可取得邊的 information matrix。

---

## 模組三：ALC Candidate Construction（Ref1 Alg.1 + Ref2 §4.1）

**輸入**：RTAbMap pose graph G、lighthouse 集合 L、當前 pose p_r  
**輸出**：ALC candidate 集合 T_alc

### Lighthouse 建立（主動，Ref1 §V-A）

```cpp
// 抵達 S_L > cs_lighthouse 的位置，且距上次 lighthouse > d_min 時觸發
void tryCreateLighthouse(const Keyframe& current) {
    if (current.saliency_local > cs_lighthouse &&
        distanceTo(last_lighthouse_pose_) > d_min_lighthouse) {
        publishRotateInPlace();    // 原地旋轉 360°
        // 旋轉完成後，將新增的 keyframes 登記為 lighthouse
    }
}
```

### Passive Candidate 建立（DBSCAN 聚類，Ref2 §4.1 + Ref1 Alg.1）

**改進**：以 DBSCAN 取代 Ref1 的固定半徑聚類，能自適應地合併空間鄰近的 keyframe，對不規則環境更穩健。

```cpp
// 篩選條件（Ref1 Alg.1 line 7）
bool shouldInclude(const Keyframe& vi, const Pose6f& p_r) {
    return euclidean(p_r, vi.pose) <= cE          // 距離上限
        && vi.saliency_local >= cs                // local saliency 下限
        && graphDist(p_r, vi.node_id) >= cG;      // graph distance 下限（uncertainty 足夠大）
}

// DBSCAN 聚類 → 每個 cluster 選出 representative
std::vector<ALCCandidate> buildCandidates(
    const PoseGraph& G, const Pose6f& p_r)
{
    auto filtered = filterKeyframes(G, p_r);    // 套用上述篩選
    auto clusters = dbscan(filtered, eps, min_pts);
    std::vector<ALCCandidate> candidates;
    for (auto& cluster : clusters) {
        ALCCandidate tau;
        tau.rep_pose = selectRepresentative(cluster).pose;
        tau.keyframe_ids = cluster.node_ids;
        tau.euclidean_dist = euclidean(p_r, tau.rep_pose);  // cheap，不需 A*
        candidates.push_back(tau);
    }
    return candidates;
}
```

### 超參數

| 參數               | 意義                        | 初始值 |
| ------------------ | --------------------------- | ------ |
| `cE`               | 最大候選距離                | 15.0 m |
| `cG`               | 最小 graph distance         | 3.0 m  |
| `cs`               | 最小 local saliency         | 0.3    |
| `cs_lighthouse`    | 觸發 lighthouse 的 saliency | 0.6    |
| `d_min_lighthouse` | Lighthouse 最小間距         | 5.0 m  |
| `eps`              | DBSCAN 鄰域半徑             | 1.5 m  |
| `min_pts`          | DBSCAN 最小點數             | 2      |

---

## 模組四：Loop Closure Probability（P_LC）

### 統一 S_L + S_G 的 P_LC 公式

RTAbMap 的 loop closure 是 appearance-based：成功需要同時滿足兩件事：
1. **Pairwise feature matching 能成功**：目標 keyframe 有足夠的局部特徵（S_L）
2. **Place retrieval 能找到正確 keyframe**：目標在 BoW 詞典中足夠稀有，不被其他相似場景混淆（S_G）

原始 Ref1 公式只有 S_L 一個因子（tanh 項）。Ref2 同樣只處理 local saliency。本計畫將兩者統一為：

```
P_LC_intrinsic(v) = tanh(cv_L × S_L(v)) × tanh(cv_G × S_G(v))

l_minus(v) = graph_dist(v_r, v) + map_dist(v_r, v)   ← 見模組二的 surrogate 說明

P_LC(v) = P_LC_intrinsic(v) × exp(−l_minus² / cl²)
```

- **tanh(cv_L × S_L)**：pairwise matching 機率（texture richness）
- **tanh(cv_G × S_G)**：place retrieval 機率（scene rarity）
- **exp(−l_minus²/cl²)**：uncertainty 衰減（相對不確定性越大，loop closure 越難成功）
- 三項乘積：S_L 高 S_G 低 → 匹配容易但場景不獨特（可能找錯地方）；S_G 高 S_L 低 → 獨特但特徵稀少（匹配困難）；兩者都要高才是好候選點

**P_LC_intrinsic 是純 appearance quality 指標**（不含距離項），直接用於 representative 選取（見模組一代表點選擇一節）。

### Cluster P_LC（Ref1 Eq.9）

```
P_LC(τ) = 1 − ∏ᵢ (1 − P_LC(vᵢ))
```

### 設計注意：P_LC 的系統性高估

Ref1 的 cluster P_LC 乘法公式默帶獨立性假設。實際上空間相鄰的 keyframe 共享環境條件（紋理、光線、場景結構），回環事件**正相關**：景象好時大家都成功，景象差時大家都失敗。

正相關時，`P(所有人失敗)` 高於獨立假設，因此：

```
P(至少一個成功) = 1 − P(所有人失敗)
               < 1 − Π(1 − P_LC(vᵢ))   ← 公式值（獨立假設）
```

**乘法公式系統性高估 cluster P_LC**。v0.1 沿用此公式，但只當作相對排序的 ranking signal，不當作機率上界。

### Ref2 的 empirical P_L（作為 future reference）

Ref2 以 prior mission data 訓練雙 saliency surface model。在 indoor 探索中無 prior data，但概念被吸收進上方的 `P_LC_intrinsic` 雙因子乘積結構。

---

## 模組五：Reward Function（Ref1 Eq.10）

### ALC Reward（Ref1 §V-C）

R(τ) 只回答一個問題：**給定我們要做 ALC，去哪個候選點最好？** Coverage 不在此決策層。

```
R(τ) = −ct × l(p_r, p_τ, M)  +  P_LC(τ) × ΔU(τ)
         └─ 旅行成本（A* map dist）    └─ 期望 uncertainty reduction
```

兩個輸入都需要 A*：
- `l(p_r, p_τ, M)`：A* map distance（旅行成本）
- `ΔU(τ) = l(p_r, p_τ, G) − l(p_r, p_τ, M)`：其中 `l(p_r, p_τ, M)` 同一次 A* 可複用
- `P_LC(τ)`：需要 `map_dist` 計算 `l_minus`

**Coverage 的角色**：Coverage 狀態（A_covered / A_target）影響「現在要不要暫停去做 ALC」的 trigger 判斷，不進入 R(τ) 的計算（見模組八 Trigger）。

### Reward Upper Bound（Ref1 §V-D.1）

為讓 BNB 在不跑 A* 的情況下得到合法 upper bound，**所有 `map_dist` 出現都替換成 `euclidean_dist`**（因為 `euclidean_dist ≤ map_dist`）：

```
ΔU_ub(τ)     = l(p_r, p_τ, G) − d(p_r, p_τ)
             ≥ l(p_r, p_τ, G) − l(p_r, p_τ, M) = ΔU(τ)  ✓

l_minus_ub   = l(p_r, p_τ, G) + d(p_r, p_τ)
             ≤ l(p_r, p_τ, G) + l(p_r, p_τ, M) = l_minus
→ exp(−l_minus_ub²/cl²) ≥ exp(−l_minus²/cl²)

P_LC_ub(τ)  = tanh(cv_L × S_L_max) × tanh(cv_G × S_G_max) × exp(−l_minus_ub² / cl²)
             ≥ P_LC(τ)  ✓
  （S_L_max、S_G_max 取 cluster 中各 keyframe 的最大值，保證上界）

R_ub(τ)     = −ct × d(p_r, p_τ)  +  P_LC_ub(τ) × ΔU_ub(τ)
             ≥ R(τ)  ✓
```

**所有量均 cheap（O(1)，只需 graph_dist 和 euclidean_dist，不跑 A*）**，是合法的 BNB upper bound。

舊版公式 `R_ub = −ct × d + P_LC(τ) × ΔU(τ)` 是錯的：`P_LC` 和 `ΔU` 本身需要 A*，不是 cheap quantity。

---

## 模組六：BNB Target Selection（Ref1 Alg.2）

`computeRewardUB` 只用 cheap quantities（graph_dist、euclidean_dist），不跑 A*：

```cpp
void computeRewardUB(ALCCandidate& c, const Pose6f& robot_pose, Params p) {
    // graph_dist 和 euclidean_dist 在 candidate 建立時已計算
    c.delta_U_ub = c.graph_dist - c.euclidean_dist;                   // ΔU upper bound
    float l_minus_ub = c.graph_dist + c.euclidean_dist;               // l_minus lower bound
    // cluster 中 S_L_max、S_G_max：作為 P_LC_ub 的 appearance upper bound
    float sl_max = 0.f, sg_max = 0.f;
    for (int id : c.keyframe_ids) {
        sl_max = std::max(sl_max, keyframes_[id].saliency_local);
        sg_max = std::max(sg_max, keyframes_[id].saliency_global);
    }
    c.P_lc_ub = std::tanh(p.cv_L * sl_max)
              * std::tanh(p.cv_G * sg_max)
              * std::exp(-(l_minus_ub * l_minus_ub) / (p.cl * p.cl)); // P_LC upper bound
    c.reward_ub = -p.ct * c.euclidean_dist + c.P_lc_ub * c.delta_U_ub;
}
```

```cpp
ALCCandidate selectTarget(std::vector<ALCCandidate>& candidates,
                          const Pose6f& robot_pose)
{
    // 對所有候選點計算 R_ub（全 cheap，不跑 A*）
    for (auto& c : candidates)
        computeRewardUB(c, robot_pose, params_);

    // 初始化：選 R_ub 最大的候選點，計算真實 reward
    auto it = std::max_element(candidates.begin(), candidates.end(),
        [](auto& a, auto& b){ return a.reward_ub < b.reward_ub; });
    it->map_dist  = runAstar(robot_pose, it->rep_pose);
    it->reward    = computeReward(*it);
    ALCCandidate best = *it;
    candidates.erase(it);

    while (!candidates.empty()) {
        // 淘汰：R_ub < 當前最佳 reward，無法超越
        candidates.erase(
            std::remove_if(candidates.begin(), candidates.end(),
                [&](auto& c){ return c.reward_ub < best.reward; }),
            candidates.end());

        if (candidates.empty()) break;

        // 下一個最有希望的候選點
        auto next = std::max_element(candidates.begin(), candidates.end(),
            [](auto& a, auto& b){ return a.reward_ub < b.reward_ub; });
        next->map_dist = runAstar(robot_pose, next->rep_pose);
        next->reward   = computeReward(*next);
        if (next->reward > best.reward)
            best = *next;
        candidates.erase(next);
    }
    return best;
}
```

Ref1 實驗（§VII Fig.4）：平均 2.36 次 A* 從 13.70 個候選點中找到最優解（節省約 83% 計算量）。

---

## 模組七：Saliency-Weighted A*（Ref2 §4.2，後期優化）

Ref2 在路徑規劃中讓路徑偏向 salient 區域，增加沿途看到已知場景的機率：

```
d(xi, xk) = w(S_L_k) × euclidean(xi, xk)
w(S_L) = 2 − S_L      // S_L=0 → double cost；S_L=1 → no penalty
```

v0.1 先用標準 A*；待 saliency map 穩定後再替換 heuristic。

---

## 模組八：Triggering Conditions（Ref1 §VI-A）

### Adaptive Threshold（Ref1 改進核心）

Trigger 條件看 **R(τ*)**（機率加權的期望 uncertainty reduction 扣掉旅行成本），不只看 ΔU。只看 ΔU 會把「ΔU 大但 P_LC 很低」的候選點也觸發，浪費旅行時間。

Coverage 狀態可以影響 threshold 本身（frontier 還多時容忍 uncertainty 更高，提升 threshold）：

```cpp
bool shouldTriggerALC(const ALCCandidate& best_tau,
                      rclcpp::Time now, rclcpp::Time last_alc,
                      float coverage_ratio)           // A_covered / A_target ∈ [0,1]
{
    double elapsed   = (now - last_alc).seconds();
    // 時間衰減：距上次 ALC 越久，threshold 越低（Ref1 §VI-A）
    double threshold = theta_max * std::exp(-lambda_decay * elapsed);
    // Coverage 調節：覆蓋率越低（還有很多 frontier），threshold 適度提升
    threshold *= (1.0 + alpha_cov * (1.0 - coverage_ratio));

    return best_tau.reward > threshold;   // 用 R(τ*)，不用 ΔU
}
```

- `best_tau.reward`：BNB 計算完成後的真實 R(τ*)，已含 P_LC 加權
- Coverage 調節只改 threshold 斜率，不改 reward 的計算
- 距上次 ALC 越久，或覆蓋率越高（frontier 快用完），越容易觸發

---

## 模組九：SLAM-First Decision Loop

舊的設計是 Frontier-first 主迴圈 + ALC 插隊，與「ALC 是探索品質內生組成」的核心主張衝突。新設計讓 **SLAMGraphPlanner 是決策主體**，frontier expansion 是它評估的選項之一。

```
States: EVALUATING | EXEC_FRONTIER | EXEC_ALC_NAVIGATE | EXEC_ALC_ROTATE
      | EXEC_EXPAND_ANCHOR | EXEC_CLOSE_GAP | REFINEMENT

─────────────────────────────────────────────────────────
EVALUATING（每個 goal 完成或 timeout 後進入）:
    1. 更新 pose graph state（uncertainty、topology）
    2. buildCandidates() → BNB → τ*，計算 R(τ*)
    3. 計算各 motion option 的 utility：

       U_alc        = R(τ*)                       （若 shouldTriggerALC() 才為正值）
       U_frontier   = frontier_info_gain(best_frontier)
       U_anchor_exp = info_gain_near_anchor(best_anchor)
       U_close_gap  = gap_closure_reward(best_gap)

    4. 選 utility 最高的 option → 對應 EXEC_* state

─────────────────────────────────────────────────────────
EXEC_FRONTIER:
    publish navigate_to_pose(best_frontier)
    on arrival / timeout → EVALUATING

EXEC_ALC_NAVIGATE:
    publish navigate_to_pose(τ*.rep_pose)
    on success → EXEC_ALC_ROTATE
    on nav fail → 標記 τ* 不可達，→ EVALUATING

EXEC_ALC_ROTATE:
    publish rotate_in_place(360°)
    wait for /rtabmap/info loopClosureId > 0 OR timeout
    last_alc_time = now()
    → EVALUATING

EXEC_EXPAND_ANCHOR / EXEC_CLOSE_GAP:
    （v0.1 stub，保留接口，暫時 fallback 至 EXEC_FRONTIER）
    → EVALUATING

─────────────────────────────────────────────────────────
REFINEMENT（探索終止後，Ref1 §VI）:
    path coverage planner 進一步穩定 pose graph
```

**與舊設計的關鍵差異**：EVALUATING 每次都計算所有 option 的 utility 並比較，不存在「預設是 frontier」的偏見。若 frontier utility 最高，就做 frontier；若 ALC reward 最高（且 trigger 條件成立），就做 ALC。

---

## RTAbMap 整合

### 訂閱

| Topic                | 用途                                                                 |
| -------------------- | -------------------------------------------------------------------- |
| `/rtabmap/mapData`   | Pose graph（nodes, edges, links）                                    |
| `/rtabmap/info`      | loop closure 事件（`loopClosureId > 0`），及 word count for saliency |
| `/rtabmap/odom_info` | Covariance（Neighbor D-optimality 用）                               |
| `/map`               | Occupancy map，A* 使用                                               |

### 發布

| Topic / Action            | 用途                  |
| ------------------------- | --------------------- |
| Nav2 `navigate_to_pose`   | 導航至 τ*             |
| 自訂 `rotate_in_place`    | 原地 360° 旋轉        |
| `/alc_planner/status`     | Planner 狀態（debug） |
| `/alc_planner/candidates` | 候選點視覺化（RViz）  |

### Saliency Proxy

RTAbMap 無直接對應 Ref2 的 saliency 輸出，替代方案：

| Saliency        | Proxy 與說明                                                                                                                                                                                                                                                            |
| --------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `S_L`（local）  | **`/rtabmap/info.wordsRecognized`**（當前幀在 BoW 詞典中命中的詞數）正規化。不用 `wordsMatchedPrev`：後者是「與上一幀的匹配數」，是時序相對量（靜止不動時偏高、轉向後驟降），無法反映影像本身的紋理豐富程度。`wordsRecognized` 是影像對詞典的命中數，是影像的內在屬性。 |
| `S_G`（global） | Word frequency 倒置：`S_G ∝ 1 / mean_word_freq_in_cluster`。RTAbMap 維護 BoW 詞典，每個 word 有全局出現次數，頻率低的 word 代表稀有視覺場景，對應高 S_G。                                                                                                               |

---

## 實作順序

```
Phase 1 — Skeleton
  □ ALCPlannerNode：訂閱 mapData, info, map
  □ Pose graph Dijkstra（graph distance）
  □ Saliency proxy（S_L + S_G）

Phase 2 — Candidate Construction
  □ DBSCAN 聚類
  □ Representative 選取（S_G + uncertainty）
  □ Lighthouse 觸發與記錄

Phase 3 — Metrics & P_LC
  □ Distance-Based ΔU（Ref1 Eq.8）
  □ P_LC heuristic（Ref1 Eq.9）
  □ R(τ) 與 R_ub(τ)

Phase 4 — BNB Target Selection
  □ Nav2 compute_path（A* map distance）
  □ BNB 選取 τ*

Phase 5 — State Machine & Triggering
  □ Adaptive threshold
  □ Frontier ↔ ALC 切換
  □ 360° rotation

Phase 6 — 擴充
  □ Coverage ratio 精化：改善 A_covered 的估計精度（目前用 visited cell count）
  □ Saliency-weighted A*（Ref2）
  □ Neighbor D-optimality（若 RTAbMap 能提供 covariance）
  □ P_LC empirical model（累積 loop closure 歷史後訓練）
```

---

## 論文方法對照 Checklist

| 來源 | 章節          | 方法                                                           | 本計畫模組        |
| ---- | ------------- | -------------------------------------------------------------- | ----------------- |
| Ref1 | §IV-B, Eq.8   | Distance-Based ΔU                                              | 模組二 2-A        |
| Ref1 | §IV-A, Eq.5-6 | Neighbor D-optimality                                          | 模組二 2-B        |
| Ref1 | §V-A, Alg.1   | ALC Candidate Construction                                     | 模組三            |
| Ref1 | §V-B, Eq.9    | P_LC heuristic                                                 | 模組四            |
| Ref1 | §V-C, Eq.10   | Reward function                                                | 模組五            |
| Ref1 | §V-D.1        | Reward upper bound R_ub                                        | 模組五            |
| Ref1 | §V-D.2, Alg.2 | Branch and Bound selection                                     | 模組六            |
| Ref1 | §VI-A         | Adaptive triggering threshold                                  | 模組八            |
| Ref1 | §VI           | Frontier ↔ ALC state machine                                   | 模組九            |
| Ref2 | §3            | Dual saliency（S_L + S_G）                                     | 模組一            |
| Ref2 | §4.1          | DBSCAN clustering + rep selection                              | 模組三            |
| Ref2 | §4.2          | Saliency-weighted A*                                           | 模組七（Phase 6） |
| Ref2 | §4.3.3        | Coverage term A_map（作為 trigger 調節因子，不入 reward）      | 模組八            |
| Ref2 | §4.3.4        | Reward as weighted sum（α 概念：coverage 調節 threshold 斜率） | 模組八            |

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
