/**
 * expression_evaluator.hpp - 策略表达式求值器
 */
#pragma once

#include <string>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <chrono>
#include <regex>

namespace drone_control {
namespace access_control {

class ExpressionEvaluator {
public:
    using BuiltinFunction = std::function<bool(const std::vector<std::string>&)>;
    
    ExpressionEvaluator();
    ~ExpressionEvaluator() = default;
    
    /** 在给定上下文中求值布尔表达式 */
    bool evaluate(const std::string& expression, 
                  const std::map<std::string, std::string>& context);
    
    /** 注册自定义内置函数 */
    void registerFunction(const std::string& name, BuiltinFunction func);
    
    /** 校验表达式语法是否合法 */
    bool validateExpression(const std::string& expression);

private:
    std::map<std::string, BuiltinFunction> builtin_functions_;
    const std::map<std::string, std::string>* current_context_;
    
    // Expression parsing and evaluation helpers
    std::string substituteAttributes(const std::string& expression,
                                   const std::map<std::string, std::string>& context);
    bool evaluateSimpleExpression(const std::string& expression);
    bool evaluateFunctionCall(const std::string& function_name,
                            const std::vector<std::string>& args);
    /** 若 token 为 func(...) 形式则求值并写回 *out_result，返回是否已求值 */
    bool tryEvaluateTokenAsFunction(const std::string& token, bool* out_result);
    
    // Built-in function implementations
    void registerBuiltinFunctions();
    
    // String and parsing utilities
    std::vector<std::string> tokenize(const std::string& expression);
    std::string trim(const std::string& str);
    bool isQuotedString(const std::string& str);
    std::string unquote(const std::string& str);
    
    // Comparison operators
    bool compareValues(const std::string& left, const std::string& op, const std::string& right);
    bool isNumeric(const std::string& str);
    double toNumeric(const std::string& str);
};

} // namespace access_control
} // namespace drone_control