(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.Rss1uExpression = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const DEG = Math.PI / 180;

  function plainNumberInfo(rawText) {
    const text = String(rawText).trim();
    const match = text.match(/^[+-]?\d+(?:\.(\d*))?$/);
    if (!match) return null;
    const decimals = match[1]?.length || 0;
    return {
      value: Number(text),
      decimals,
      step: decimals === 0 ? 1 : 10 ** (-decimals),
      stepText: decimals === 0 ? "1" : `0.${"0".repeat(decimals - 1)}1`,
    };
  }

  function formatNumber(value) {
    if (!Number.isFinite(value)) return "";
    return Number(value).toPrecision(12).replace(/(?:\.0+|(?:(\.\d*?)0+))$/, "$1");
  }

  function evaluate(source, options = {}) {
    const text = String(source).trim().replace(/[×·]/g, "*").replace(/÷/g, "/");
    if (!text) throw new Error("请输入数值或公式");
    let position = 0;

    const make = (value, degree = false, radian = false) => ({ value, degree, radian });
    const skipSpaces = () => {
      while (/\s/.test(text[position] || "")) position += 1;
    };
    const fail = (message) => {
      throw new Error(`${message}（第 ${Math.min(position + 1, text.length)} 个字符附近）`);
    };
    const consume = (token) => {
      skipSpaces();
      if (text.startsWith(token, position)) {
        position += token.length;
        return true;
      }
      return false;
    };

    function applyFunction(name, argument) {
      const angle = argument.degree || !argument.radian
        ? argument.value * DEG
        : argument.value;
      if (name === "sin") return make(Math.sin(angle));
      if (name === "cos") return make(Math.cos(angle));
      if (name === "tan") return make(Math.tan(angle));
      if (name === "sqrt") {
        if (argument.value < 0) fail("sqrt 的参数不能小于 0");
        return make(Math.sqrt(argument.value));
      }
      if (name === "abs") return make(
        Math.abs(argument.value), argument.degree, argument.radian,
      );
      fail(`不支持函数 ${name}`);
      return make(NaN);
    }

    function parsePrimary() {
      skipSpaces();
      let result;
      if (consume("(")) {
        result = parseExpression();
        if (!consume(")")) fail("缺少右括号 )");
      } else if (text[position] === "π") {
        position += 1;
        result = make(Math.PI, false, true);
      } else {
        const remaining = text.slice(position);
        const numberMatch = remaining.match(/^(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?/);
        if (numberMatch) {
          position += numberMatch[0].length;
          result = make(Number(numberMatch[0]));
        } else {
          const identifierMatch = remaining.match(/^[A-Za-z_][A-Za-z0-9_]*/);
          if (!identifierMatch) fail("需要数字、常量或函数");
          const name = identifierMatch[0].toLowerCase();
          position += identifierMatch[0].length;
          if (name === "pi") result = make(Math.PI, false, true);
          else if (name === "e") result = make(Math.E);
          else {
            if (!consume("(")) fail(`函数 ${name} 后需要左括号 (`);
            const argument = parseExpression();
            if (!consume(")")) fail(`函数 ${name} 缺少右括号 )`);
            result = applyFunction(name, argument);
          }
        }
      }
      skipSpaces();
      if (consume("°")) result.degree = true;
      return result;
    }

    function parsePower() {
      const left = parsePrimary();
      if (!consume("^")) return left;
      const right = parseUnary();
      return make(left.value ** right.value, false, left.radian || right.radian);
    }

    function parseUnary() {
      if (consume("+")) return parseUnary();
      if (consume("-")) {
        const value = parseUnary();
        return make(-value.value, value.degree, value.radian);
      }
      return parsePower();
    }

    function parseTerm() {
      let result = parseUnary();
      while (true) {
        if (consume("*")) {
          const right = parseUnary();
          result = make(
            result.value * right.value,
            result.degree !== right.degree,
            result.radian || right.radian,
          );
        } else if (consume("/")) {
          const right = parseUnary();
          if (right.value === 0) fail("不能除以 0");
          result = make(
            result.value / right.value,
            result.degree && !right.degree,
            result.radian || right.radian,
          );
        } else break;
      }
      return result;
    }

    function parseExpression() {
      let result = parseTerm();
      while (true) {
        if (consume("+")) {
          const right = parseTerm();
          result = make(
            result.value + right.value,
            result.degree || right.degree,
            result.radian || right.radian,
          );
        } else if (consume("-")) {
          const right = parseTerm();
          result = make(
            result.value - right.value,
            result.degree || right.degree,
            result.radian || right.radian,
          );
        } else break;
      }
      return result;
    }

    const result = parseExpression();
    skipSpaces();
    if (position !== text.length) fail(`无法识别“${text.slice(position)}”`);
    let value = result.value;
    if (options.angleField && result.radian && !result.degree) value /= DEG;
    else if (!options.angleField && result.degree) value *= DEG;
    if (!Number.isFinite(value)) throw new Error("公式结果不是有限数值");
    return value;
  }

  return { evaluate, formatNumber, plainNumberInfo };
});
