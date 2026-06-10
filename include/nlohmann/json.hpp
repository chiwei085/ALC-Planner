#pragma once

#include <cstddef>
#include <initializer_list>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace nlohmann
{

class json
{
public:
    class parse_error : public std::runtime_error
    {
    public:
        explicit parse_error(const std::string& message)
            : std::runtime_error(message) {}
    };

    using object_t = std::map<std::string, json>;
    using array_t = std::vector<json>;

    json() : value_(nullptr) {}
    json(std::nullptr_t) : value_(nullptr) {}
    json(bool value) : value_(value) {}
    json(int value) : value_(static_cast<double>(value)) {}
    json(unsigned int value) : value_(static_cast<double>(value)) {}
    json(long value) : value_(static_cast<double>(value)) {}
    json(unsigned long value) : value_(static_cast<double>(value)) {}
    json(long long value) : value_(static_cast<double>(value)) {}
    json(unsigned long long value) : value_(static_cast<double>(value)) {}
    json(double value) : value_(value) {}
    json(const char* value) : value_(std::string(value)) {}
    json(std::string value) : value_(std::move(value)) {}
    json(array_t value) : value_(std::move(value)) {}
    json(object_t value) : value_(std::move(value)) {}

    json(std::initializer_list<std::pair<const std::string, json>> fields)
        : value_(object_t{}) {
        auto& object = std::get<object_t>(value_);
        for (const auto& field : fields) {
            object[field.first] = field.second;
        }
    }

    static json object() { return json(object_t{}); }

    static json array() { return json(array_t{}); }

    static json array(std::initializer_list<json> values) {
        return json(array_t(values.begin(), values.end()));
    }

    static json parse(const std::string& text) {
        Parser parser(text);
        json result = parser.parseValue();
        parser.skipSpace();
        if (!parser.done()) {
            throw parse_error("unexpected trailing JSON content");
        }
        return result;
    }

    bool contains(const std::string& key) const {
        if (!std::holds_alternative<object_t>(value_)) {
            return false;
        }
        const auto& object = std::get<object_t>(value_);
        return object.find(key) != object.end();
    }

    json& operator[](const std::string& key) {
        if (!std::holds_alternative<object_t>(value_)) {
            value_ = object_t{};
        }
        return std::get<object_t>(value_)[key];
    }

    const json& operator[](const std::string& key) const {
        static const json null_json;
        if (!std::holds_alternative<object_t>(value_)) {
            return null_json;
        }
        const auto& object = std::get<object_t>(value_);
        const auto it = object.find(key);
        return it == object.end() ? null_json : it->second;
    }

    void push_back(json value) {
        if (!std::holds_alternative<array_t>(value_)) {
            value_ = array_t{};
        }
        std::get<array_t>(value_).push_back(std::move(value));
    }

    std::string dump() const {
        if (std::holds_alternative<std::nullptr_t>(value_)) {
            return "null";
        }
        if (std::holds_alternative<bool>(value_)) {
            return std::get<bool>(value_) ? "true" : "false";
        }
        if (std::holds_alternative<double>(value_)) {
            return dumpNumber(std::get<double>(value_));
        }
        if (std::holds_alternative<std::string>(value_)) {
            return "\"" + escape(std::get<std::string>(value_)) + "\"";
        }
        if (std::holds_alternative<array_t>(value_)) {
            std::string output = "[";
            const auto& array = std::get<array_t>(value_);
            for (std::size_t i = 0; i < array.size(); ++i) {
                if (i != 0) {
                    output += ",";
                }
                output += array[i].dump();
            }
            output += "]";
            return output;
        }
        std::string output = "{";
        const auto& object = std::get<object_t>(value_);
        bool first = true;
        for (const auto& [key, value] : object) {
            if (!first) {
                output += ",";
            }
            first = false;
            output += "\"" + escape(key) + "\":" + value.dump();
        }
        output += "}";
        return output;
    }

    operator double() const {
        if (!std::holds_alternative<double>(value_)) {
            return 0.0;
        }
        return std::get<double>(value_);
    }

private:
    using value_t = std::variant<std::nullptr_t, bool, double, std::string,
                                 array_t, object_t>;

    class Parser
    {
    public:
        explicit Parser(const std::string& text) : text_(text) {}

        json parseValue() {
            skipSpace();
            if (done()) {
                throw parse_error("empty JSON input");
            }
            if (peek() == '{') {
                return parseObject();
            }
            if (peek() == '[') {
                return parseArray();
            }
            if (peek() == '"') {
                return json(parseString());
            }
            if (match("true")) {
                return json(true);
            }
            if (match("false")) {
                return json(false);
            }
            if (match("null")) {
                return json(nullptr);
            }
            return parseNumber();
        }

        void skipSpace() {
            while (!done() && (peek() == ' ' || peek() == '\n' ||
                               peek() == '\r' || peek() == '\t')) {
                ++pos_;
            }
        }

        bool done() const { return pos_ >= text_.size(); }

    private:
        char peek() const { return text_[pos_]; }

        char get() {
            if (done()) {
                throw parse_error("unexpected end of JSON");
            }
            return text_[pos_++];
        }

        bool match(const char* token) {
            const std::string word(token);
            if (text_.compare(pos_, word.size(), word) != 0) {
                return false;
            }
            pos_ += word.size();
            return true;
        }

        json parseObject() {
            object_t object;
            expect('{');
            skipSpace();
            if (!done() && peek() == '}') {
                ++pos_;
                return json(object);
            }
            while (true) {
                skipSpace();
                const std::string key = parseString();
                skipSpace();
                expect(':');
                object[key] = parseValue();
                skipSpace();
                const char separator = get();
                if (separator == '}') {
                    break;
                }
                if (separator != ',') {
                    throw parse_error("expected object separator");
                }
            }
            return json(object);
        }

        json parseArray() {
            array_t array;
            expect('[');
            skipSpace();
            if (!done() && peek() == ']') {
                ++pos_;
                return json(array);
            }
            while (true) {
                array.push_back(parseValue());
                skipSpace();
                const char separator = get();
                if (separator == ']') {
                    break;
                }
                if (separator != ',') {
                    throw parse_error("expected array separator");
                }
            }
            return json(array);
        }

        std::string parseString() {
            expect('"');
            std::string value;
            while (!done()) {
                const char c = get();
                if (c == '"') {
                    return value;
                }
                if (c != '\\') {
                    value += c;
                    continue;
                }
                const char escaped = get();
                switch (escaped) {
                    case '"':
                    case '\\':
                    case '/':
                        value += escaped;
                        break;
                    case 'n':
                        value += '\n';
                        break;
                    case 'r':
                        value += '\r';
                        break;
                    case 't':
                        value += '\t';
                        break;
                    default:
                        throw parse_error("unsupported JSON escape");
                }
            }
            throw parse_error("unterminated JSON string");
        }

        json parseNumber() {
            const std::size_t start = pos_;
            if (!done() && peek() == '-') {
                ++pos_;
            }
            while (!done() && peek() >= '0' && peek() <= '9') {
                ++pos_;
            }
            if (!done() && peek() == '.') {
                ++pos_;
                while (!done() && peek() >= '0' && peek() <= '9') {
                    ++pos_;
                }
            }
            return json(std::stod(text_.substr(start, pos_ - start)));
        }

        void expect(const char expected) {
            if (get() != expected) {
                throw parse_error("unexpected JSON token");
            }
        }

        const std::string& text_;
        std::size_t pos_ = 0;
    };

    static std::string escape(const std::string& value) {
        std::string output;
        for (const char c : value) {
            switch (c) {
                case '\\':
                    output += "\\\\";
                    break;
                case '"':
                    output += "\\\"";
                    break;
                case '\n':
                    output += "\\n";
                    break;
                case '\r':
                    output += "\\r";
                    break;
                case '\t':
                    output += "\\t";
                    break;
                default:
                    output += c;
                    break;
            }
        }
        return output;
    }

    static std::string dumpNumber(double value) {
        std::string text = std::to_string(value);
        while (text.size() > 1 && text.back() == '0') {
            text.pop_back();
        }
        if (!text.empty() && text.back() == '.') {
            text.pop_back();
        }
        return text;
    }

    value_t value_;
};

}  // namespace nlohmann
