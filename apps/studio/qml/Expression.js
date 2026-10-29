.pragma library

var DEG = Math.PI / 180.0

function plainNumberInfo(rawText) {
    var text = String(rawText).trim()
    var match = text.match(/^[+-]?\d+(?:\.(\d*))?$/)
    if (!match)
        return null
    var decimals = match[1] ? match[1].length : 0
    return {
        value: Number(text),
        decimals: decimals,
        step: decimals === 0 ? 1 : Math.pow(10, -decimals)
    }
}

function formatNumber(value) {
    if (!Number.isFinite(value))
        return ""
    return Number(value).toPrecision(12).replace(/(?:\.0+|(?:(\.\d*?)0+))$/, "$1")
}

function evaluate(source, angleField) {
    var text = String(source).trim().replace(/[×·]/g, "*").replace(/÷/g, "/")
    if (!text)
        throw new Error("请输入数值或公式")
    var position = 0

    function make(value, degree, radian) {
        return { value: value, degree: Boolean(degree), radian: Boolean(radian) }
    }
    function skipSpaces() {
        while (/\s/.test(text[position] || ""))
            position += 1
    }
    function fail(message) {
        throw new Error(message + "（第 " + Math.min(position + 1, text.length) + " 个字符附近）")
    }
    function consume(token) {
        skipSpaces()
        if (text.slice(position, position + token.length) === token) {
            position += token.length
            return true
        }
        return false
    }
    function applyFunction(name, argument) {
        var angle = argument.degree || !argument.radian
                  ? argument.value * DEG : argument.value
        if (name === "sin") return make(Math.sin(angle))
        if (name === "cos") return make(Math.cos(angle))
        if (name === "tan") return make(Math.tan(angle))
        if (name === "sqrt") {
            if (argument.value < 0) fail("sqrt 的参数不能小于 0")
            return make(Math.sqrt(argument.value))
        }
        if (name === "abs")
            return make(Math.abs(argument.value), argument.degree, argument.radian)
        fail("不支持函数 " + name)
        return make(NaN)
    }
    function parsePrimary() {
        skipSpaces()
        var result
        if (consume("(")) {
            result = parseExpression()
            if (!consume(")")) fail("缺少右括号 )")
        } else if (text[position] === "π") {
            position += 1
            result = make(Math.PI, false, true)
        } else {
            var remaining = text.slice(position)
            var numberMatch = remaining.match(/^(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?/)
            if (numberMatch) {
                position += numberMatch[0].length
                result = make(Number(numberMatch[0]))
            } else {
                var identifierMatch = remaining.match(/^[A-Za-z_][A-Za-z0-9_]*/)
                if (!identifierMatch) fail("需要数字、常量或函数")
                var name = identifierMatch[0].toLowerCase()
                position += identifierMatch[0].length
                if (name === "pi") result = make(Math.PI, false, true)
                else if (name === "e") result = make(Math.E)
                else {
                    if (!consume("(")) fail("函数 " + name + " 后需要左括号 (")
                    var argument = parseExpression()
                    if (!consume(")")) fail("函数 " + name + " 缺少右括号 )")
                    result = applyFunction(name, argument)
                }
            }
        }
        skipSpaces()
        if (consume("°")) result.degree = true
        return result
    }
    function parsePower() {
        var left = parsePrimary()
        if (!consume("^")) return left
        var right = parseUnary()
        return make(Math.pow(left.value, right.value), false, left.radian || right.radian)
    }
    function parseUnary() {
        if (consume("+")) return parseUnary()
        if (consume("-")) {
            var value = parseUnary()
            return make(-value.value, value.degree, value.radian)
        }
        return parsePower()
    }
    function parseTerm() {
        var result = parseUnary()
        while (true) {
            if (consume("*")) {
                var multiplied = parseUnary()
                result = make(result.value * multiplied.value,
                              result.degree !== multiplied.degree,
                              result.radian || multiplied.radian)
            } else if (consume("/")) {
                var divided = parseUnary()
                if (divided.value === 0) fail("不能除以 0")
                result = make(result.value / divided.value,
                              result.degree && !divided.degree,
                              result.radian || divided.radian)
            } else break
        }
        return result
    }
    function parseExpression() {
        var result = parseTerm()
        while (true) {
            if (consume("+")) {
                var added = parseTerm()
                result = make(result.value + added.value,
                              result.degree || added.degree,
                              result.radian || added.radian)
            } else if (consume("-")) {
                var subtracted = parseTerm()
                result = make(result.value - subtracted.value,
                              result.degree || subtracted.degree,
                              result.radian || subtracted.radian)
            } else break
        }
        return result
    }

    var result = parseExpression()
    skipSpaces()
    if (position !== text.length)
        fail("无法识别“" + text.slice(position) + "”")
    var value = result.value
    if (angleField && result.radian && !result.degree) value /= DEG
    else if (!angleField && result.degree) value *= DEG
    if (!Number.isFinite(value))
        throw new Error("公式结果不是有限数值")
    return value
}
