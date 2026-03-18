/**
 * 策略表达式求值
 * 解析并求值策略中的条件表达式（主体、资源、动作、环境），供 ABAC 引擎匹配规则。
 */
#include "access_control/expression_evaluator.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <ctime>
#include <iomanip>

namespace drone_control {
namespace access_control {

ExpressionEvaluator::ExpressionEvaluator() : current_context_(nullptr) {
    registerBuiltinFunctions();
}

bool ExpressionEvaluator::evaluate(const std::string& expression,
                                 const std::map<std::string, std::string>& context) {
    if (expression.empty()) {
        return false;
    }
    
    // 为内置函数设置当前上下文
    current_context_ = &context;
    
    // 处理"true"和"false"的特殊情况
    std::string trimmed = trim(expression);
    if (trimmed == "true") return true;
    if (trimmed == "false") return false;
    
    try {
        // 替换属性值
        std::string substituted = substituteAttributes(expression, context);
        
        // 计算替换后的表达式
        bool result = evaluateSimpleExpression(substituted);
        
        // 清除上下文
        current_context_ = nullptr;
        
        return result;
    } catch (const std::exception& e) {
        // 清除上下文并返回false以确保安全
        current_context_ = nullptr;
        return false;
    }
}

void ExpressionEvaluator::registerFunction(const std::string& name, BuiltinFunction func) {
    builtin_functions_[name] = func;
}

bool ExpressionEvaluator::validateExpression(const std::string& expression) {
    if (expression.empty()) {
        return false;
    }
    
    try {
        // 基本语法验证 - 检查括号是否平衡
        int paren_count = 0;
        for (char c : expression) {
            if (c == '(') paren_count++;
            else if (c == ')') paren_count--;
            if (paren_count < 0) return false;
        }
        return paren_count == 0;
    } catch (...) {
        return false;
    }
}

std::string ExpressionEvaluator::substituteAttributes(const std::string& expression,
                                                    const std::map<std::string, std::string>& context) {
    std::string result = expression;
    
    // 用属性值替换属性引用
    for (const auto& [key, value] : context) {
        std::string placeholder = key;
        size_t pos = 0;
        while ((pos = result.find(placeholder, pos)) != std::string::npos) {
            // 检查是否为完整单词（不是另一个标识符的一部分）
            bool is_whole_word = true;
            if (pos > 0 && (std::isalnum(result[pos-1]) || result[pos-1] == '_')) {
                is_whole_word = false;
            }
            if (pos + placeholder.length() < result.length() && 
                (std::isalnum(result[pos + placeholder.length()]) || result[pos + placeholder.length()] == '_')) {
                is_whole_word = false;
            }
            
            if (is_whole_word) {
                // 如果值未加引号且包含空格或特殊字符，则为其添加引号
                std::string quoted_value = value;
                if (!isQuotedString(value) && 
                    (value.find(' ') != std::string::npos || 
                     value.find(',') != std::string::npos ||
                     value.find('\'') != std::string::npos)) {
                    quoted_value = "'" + value + "'";
                }
                result.replace(pos, placeholder.length(), quoted_value);
                pos += quoted_value.length();
            } else {
                pos += placeholder.length();
            }
        }
    }
    
    return result;
}

bool ExpressionEvaluator::evaluateSimpleExpression(const std::string& expression) {
    std::string expr = trim(expression);
    
    // 首先检查是否为函数调用（在分词之前）
    size_t paren_pos = expr.find('(');
    if (paren_pos != std::string::npos && expr.back() == ')') {
        std::string func_name = trim(expr.substr(0, paren_pos));
        std::string args_str = expr.substr(paren_pos + 1, expr.length() - paren_pos - 2);
        
        std::vector<std::string> args;
        if (!args_str.empty()) {
            // 解析函数参数，处理带引号的字符串
            std::string current_arg;
            bool in_quotes = false;
            char quote_char = '\0';
            
            for (size_t i = 0; i < args_str.length(); ++i) {
                char c = args_str[i];
                
                if (!in_quotes && (c == '\'' || c == '"')) {
                    in_quotes = true;
                    quote_char = c;
                    current_arg += c;
                } else if (in_quotes && c == quote_char) {
                    in_quotes = false;
                    current_arg += c;
                    quote_char = '\0';
                } else if (!in_quotes && c == ',') {
                    args.push_back(trim(current_arg));
                    current_arg.clear();
                } else {
                    current_arg += c;
                }
            }
            
            if (!current_arg.empty()) {
                args.push_back(trim(current_arg));
            }
        }
        
        return evaluateFunctionCall(func_name, args);
    }
    
    // 处理逻辑运算符（AND, OR）
    // 目前使用简单的从左到右计算
    std::vector<std::string> tokens = tokenize(expr);
    
    if (tokens.empty()) {
        return false;
    }
    
    if (tokens.size() == 1) {
        if (tokens[0] == "true") return true;
        if (tokens[0] == "false") return false;
        bool single_result = false;
        if (tryEvaluateTokenAsFunction(tokens[0], &single_result)) return single_result;
        return false;
    }
    
    // 处理比较操作
        for (size_t i = 1; i < tokens.size() - 1; ++i) {
            if (tokens[i] == "==" || tokens[i] == "!=" || 
                tokens[i] == "<" || tokens[i] == ">" || 
                tokens[i] == "<=" || tokens[i] == ">=" ||
                tokens[i] == "IN" || tokens[i] == "NOT_IN" || 
                tokens[i] == "CONTAINS" || tokens[i] == "NOT_CONTAINS") {
            
            std::string left = tokens[i-1];
            std::string op = tokens[i];
            std::string right = tokens[i+1];
            
            bool result = compareValues(left, op, right);
            
            // 用结果替换比较操作
            tokens[i-1] = result ? "true" : "false";
            tokens.erase(tokens.begin() + i, tokens.begin() + i + 2);
            i = 0; // 从头开始重新处理
        }
    }
    
    // 处理逻辑操作（含 true/false 与 func(...) 形式的 token）
    bool result = false;
    std::string current_op = "";
    
    for (const auto& token : tokens) {
        if (token == "AND" || token == "OR") {
            current_op = token;
        } else {
            bool value = false;
            if (token == "true") value = true;
            else if (token == "false") value = false;
            else (void) tryEvaluateTokenAsFunction(token, &value);  // 非函数则保持 false
            if (current_op.empty()) {
                result = value;
            } else if (current_op == "AND") {
                result = result && value;
            } else if (current_op == "OR") {
                result = result || value;
            }
        }
    }
    return result;
}

bool ExpressionEvaluator::evaluateFunctionCall(const std::string& function_name,
                                             const std::vector<std::string>& args) {
    auto it = builtin_functions_.find(function_name);
    if (it != builtin_functions_.end()) {
        return it->second(args);
    }
    return false;
}

bool ExpressionEvaluator::tryEvaluateTokenAsFunction(const std::string& token, bool* out_result) {
    if (token.size() < 3 || out_result == nullptr) return false;
    size_t lp = token.find('(');
    if (lp == std::string::npos || token.back() != ')') return false;
    std::string name = trim(token.substr(0, lp));
    if (name.empty()) return false;
    std::string args_str = token.substr(lp + 1, token.size() - lp - 2);
    std::vector<std::string> args;
    std::string current_arg;
    bool in_quotes = false;
    char quote_char = '\0';
    for (size_t i = 0; i < args_str.size(); ++i) {
        char c = args_str[i];
        if (!in_quotes && (c == '\'' || c == '"')) {
            in_quotes = true;
            quote_char = c;
            current_arg += c;
        } else if (in_quotes && c == quote_char) {
            in_quotes = false;
            current_arg += c;
            quote_char = '\0';
        } else if (!in_quotes && c == ',') {
            args.push_back(trim(current_arg));
            current_arg.clear();
        } else {
            current_arg += c;
        }
    }
    if (!current_arg.empty()) args.push_back(trim(current_arg));
    if (builtin_functions_.find(name) == builtin_functions_.end()) return false;
    *out_result = evaluateFunctionCall(name, args);
    return true;
}

void ExpressionEvaluator::registerBuiltinFunctions() {
    // 用于去除字符串引号的辅助lambda
    auto unquote_helper = [](const std::string& str) -> std::string {
        if (str.length() >= 2 && 
            ((str.front() == '\'' && str.back() == '\'') ||
             (str.front() == '"' && str.back() == '"'))) {
            return str.substr(1, str.length() - 2);
        }
        return str;
    };
    
    // 用于去除字符串空白的辅助lambda
    auto trim_helper = [](const std::string& str) -> std::string {
        size_t start = str.find_first_not_of(" \t\n\r");
        if (start == std::string::npos) return "";
        
        size_t end = str.find_last_not_of(" \t\n\r");
        return str.substr(start, end - start + 1);
    };
    
    // time_in_range函数: time_in_range('09:00', '17:00')
    registerFunction("time_in_range", [this, unquote_helper](const std::vector<std::string>& args) -> bool {
        if (args.size() != 2) return false;
        
        try {
            std::string start_time = unquote_helper(args[0]);
            std::string end_time = unquote_helper(args[1]);
            
            // 解析开始和结束时间（HH:MM格式）
            int start_hour, start_min, end_hour, end_min;
            if (sscanf(start_time.c_str(), "%d:%d", &start_hour, &start_min) != 2 ||
                sscanf(end_time.c_str(), "%d:%d", &end_hour, &end_min) != 2) {
                return false;
            }
            
            int current_hour, current_min;
            
            // 检查上下文中是否有current_time
            if (current_context_ && current_context_->find("current_time") != current_context_->end()) {
                std::string context_time = current_context_->at("current_time");
                if (sscanf(context_time.c_str(), "%d:%d", &current_hour, &current_min) == 2) {
                    // 使用上下文时间
                } else {
                    return false;
                }
            } else {
                // 回退到系统时间
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto tm = *std::localtime(&time_t);
                current_hour = tm.tm_hour;
                current_min = tm.tm_min;
            }
            
            int current_minutes = current_hour * 60 + current_min;
            int start_minutes = start_hour * 60 + start_min;
            int end_minutes = end_hour * 60 + end_min;
            
            if (start_minutes <= end_minutes) {
                // 同一天的时间范围
                return current_minutes >= start_minutes && current_minutes <= end_minutes;
            } else {
                // 跨夜的时间范围
                return current_minutes >= start_minutes || current_minutes <= end_minutes;
            }
        } catch (...) {
            return false;
        }
    });
    
    // contains函数: contains(list, item)
    registerFunction("contains", [unquote_helper](const std::vector<std::string>& args) -> bool {
        if (args.size() != 2) return false;
        
        std::string list_str = unquote_helper(args[0]);
        std::string item = unquote_helper(args[1]);
        
        // 简单的逗号分隔列表检查
        return list_str.find(item) != std::string::npos;
    });
    
    // in_list函数: in_list(item, 'item1,item2,item3')
    registerFunction("in_list", [unquote_helper, trim_helper](const std::vector<std::string>& args) -> bool {
        if (args.size() != 2) return false;
        
        std::string item = unquote_helper(args[0]);
        std::string list_str = unquote_helper(args[1]);
        
        std::stringstream ss(list_str);
        std::string list_item;
        while (std::getline(ss, list_item, ',')) {
            if (trim_helper(list_item) == item) {
                return true;
            }
        }
        return false;
    });
    
    // is_weekend函数: is_weekend()
    registerFunction("is_weekend", [](const std::vector<std::string>& args) -> bool {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);
        
        return tm.tm_wday == 0 || tm.tm_wday == 6; // 星期日 = 0, 星期六 = 6
    });
    
    // is_emergency函数: is_emergency(status)
    registerFunction("is_emergency", [unquote_helper](const std::vector<std::string>& args) -> bool {
        if (args.size() != 1) return false;
        
        std::string status = unquote_helper(args[0]);
        return status == "emergency" || status == "critical" || status == "urgent";
    });
}

std::vector<std::string> ExpressionEvaluator::tokenize(const std::string& expression) {
    std::vector<std::string> tokens;
    std::string current_token;
    bool in_quotes = false;
    char quote_char = '\0';
    
    for (size_t i = 0; i < expression.length(); ++i) {
        char c = expression[i];
        
        if (!in_quotes && (c == '\'' || c == '"')) {
            in_quotes = true;
            quote_char = c;
            current_token += c;
        } else if (in_quotes && c == quote_char) {
            in_quotes = false;
            current_token += c;
            quote_char = '\0';
        } else if (!in_quotes && std::isspace(c)) {
            if (!current_token.empty()) {
                tokens.push_back(current_token);
                current_token.clear();
            }
        } else {
            current_token += c;
        }
    }
    
    if (!current_token.empty()) {
        tokens.push_back(current_token);
    }
    
    return tokens;
}

std::string ExpressionEvaluator::trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    
    size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

bool ExpressionEvaluator::isQuotedString(const std::string& str) {
    return (str.length() >= 2 && 
            ((str.front() == '\'' && str.back() == '\'') ||
             (str.front() == '"' && str.back() == '"')));
}

std::string ExpressionEvaluator::unquote(const std::string& str) {
    if (isQuotedString(str)) {
        return str.substr(1, str.length() - 2);
    }
    return str;
}

bool ExpressionEvaluator::compareValues(const std::string& left, const std::string& op, const std::string& right) {
    std::string left_val = unquote(trim(left));
    std::string right_val = unquote(trim(right));
    
    if (op == "==") {
        return left_val == right_val;
    } else if (op == "!=") {
        return left_val != right_val;
    } else if (op == "IN") {
        // 列表成员检查，right_val 如 ['a','b'] 或 'a,b'
        if (right_val.size() >= 2 && right_val.front() == '[' && right_val.back() == ']') {
            std::string list_content = right_val.substr(1, right_val.length() - 2);
            std::stringstream ss(list_content);
            std::string item;
            while (std::getline(ss, item, ',')) {
                std::string trimmed_item = unquote(trim(item));
                if (trimmed_item == left_val) {
                    return true;
                }
            }
            return false;
        } else {
            // 逗号分隔列表：按项精确匹配，避免子串误判（如 "a" 不应匹配 "aaa,b"）
            std::stringstream ss(right_val);
            std::string item;
            while (std::getline(ss, item, ',')) {
                if (trim(unquote(item)) == left_val) return true;
            }
            return false;
        }
    } else if (op == "NOT_IN") {
        if (right_val.size() >= 2 && right_val.front() == '[' && right_val.back() == ']') {
            std::string list_content = right_val.substr(1, right_val.length() - 2);
            std::stringstream ss(list_content);
            std::string item;
            while (std::getline(ss, item, ',')) {
                std::string trimmed_item = unquote(trim(item));
                if (trimmed_item == left_val) {
                    return false;
                }
            }
            return true;
        } else {
            std::stringstream ss(right_val);
            std::string item;
            while (std::getline(ss, item, ',')) {
                if (trim(unquote(item)) == left_val) return false;
            }
            return true;
        }
    } else if (op == "CONTAINS") {
        // 处理CONTAINS运算符用于字符串包含检查
        return left_val.find(right_val) != std::string::npos;
    } else if (op == "NOT_CONTAINS") {
        // 处理NOT_CONTAINS运算符用于字符串非包含检查
        return left_val.find(right_val) == std::string::npos;
    }
    
    // 数值比较
    if (isNumeric(left_val) && isNumeric(right_val)) {
        double left_num = toNumeric(left_val);
        double right_num = toNumeric(right_val);
        
        if (op == "<") return left_num < right_num;
        if (op == ">") return left_num > right_num;
        if (op == "<=") return left_num <= right_num;
        if (op == ">=") return left_num >= right_num;
    }
    
    // 非数值的字符串比较
    if (op == "<") return left_val < right_val;
    if (op == ">") return left_val > right_val;
    if (op == "<=") return left_val <= right_val;
    if (op == ">=") return left_val >= right_val;
    
    return false;
}

bool ExpressionEvaluator::isNumeric(const std::string& str) {
    if (str.empty()) return false;
    
    char* end;
    std::strtod(str.c_str(), &end);
    return *end == '\0';
}

double ExpressionEvaluator::toNumeric(const std::string& str) {
    return std::strtod(str.c_str(), nullptr);
}

} // namespace access_control
} // namespace drone_control