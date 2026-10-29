"use strict";

const fs = require("node:fs");
const path = require("node:path");
const model = require("../apps/visualizer/kinematics.js");
const expression = require("../apps/visualizer/expression.js");

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function validate(name, branches, options = {}) {
  const zero = model.inverse(branches, [0, 0], [0, 0]);
  if (options.expectNeutralZero !== false) {
    assert(Math.hypot(...zero.theta) < 1e-8, `${name}: neutral IK is not zero`);
  }

  const q = [0.06, -0.04];
  const inverse = model.inverse(branches, q, [0, 0]);
  const closure = model.closureError(branches, q, inverse.theta);
  assert(Math.max(...closure.map(Math.abs)) < 1e-8, `${name}: closure residual`);

  const analytical = model.jacobian(branches, q, inverse.theta).matrix;
  const numerical = [[0, 0], [0, 0]];
  const epsilon = 1e-7;
  for (let column = 0; column < 2; column += 1) {
    const plus = [...q];
    const minus = [...q];
    plus[column] += epsilon;
    minus[column] -= epsilon;
    const thetaPlus = model.inverse(branches, plus, inverse.theta).theta;
    const thetaMinus = model.inverse(branches, minus, inverse.theta).theta;
    for (let row = 0; row < 2; row += 1) {
      numerical[row][column] = (thetaPlus[row] - thetaMinus[row]) / (2 * epsilon);
    }
  }
  let maximumDifference = 0;
  for (let row = 0; row < 2; row += 1) {
    for (let column = 0; column < 2; column += 1) {
      maximumDifference = Math.max(
        maximumDifference,
        Math.abs(analytical[row][column] - numerical[row][column]),
      );
    }
  }
  assert(maximumDifference < 2e-6, `${name}: Jacobian finite-difference mismatch`);

  const forward = model.forward(branches, inverse.theta, [0.048, -0.032]);
  assert(Math.hypot(forward.q[0] - q[0], forward.q[1] - q[1]) < 1e-8, `${name}: FK/IK mismatch`);
  console.log(`${name}: passed; max closure=${Math.max(...closure.map(Math.abs))}, J diff=${maximumDifference}`);
}

validate("paper-compatible geometry", model.currentGeometry());
validate("configuration I fixed x axes", model.configurationOneGeometry());
validate("arbitrary spatial axes", model.arbitraryAxisGeometry());

const configurationCases = [
  ["validation-I.json", model.configurationOneGeometry()],
  ["validation-II.json", model.currentGeometry()],
  ["validation-III.json", model.arbitraryAxisGeometry()],
];
const comparableFields = [
  "axisPoint", "axis", "crankZero", "platformOffset", "platformPoint",
];
configurationCases.forEach(([fileName, expected]) => {
  const filePath = path.join(__dirname, "..", "configs", fileName);
  const document = JSON.parse(fs.readFileSync(filePath, "utf8"));
  assert(document.version === 2, `${fileName}: configuration version`);
  const actual = model.normalizeGeometry(document.branches);
  assert(Array.isArray(document.expressions) && document.expressions.length === 2,
    `${fileName}: expressions`);
  actual.forEach((branch, branchIndex) => {
    comparableFields.forEach((field) => branch[field].forEach((value, component) => {
      assert(Math.abs(value - expected[branchIndex][field][component]) < 1e-11,
        `${fileName}: branch ${branchIndex + 1} ${field}[${component}]`);
      const evaluated = expression.evaluate(document.expressions[branchIndex][field][component]);
      assert(Math.abs(evaluated - value) < 1e-10,
        `${fileName}: expression branch ${branchIndex + 1} ${field}[${component}]`);
    }));
    ["rodLength", "motorOffset", "motorLower", "motorUpper"].forEach((field) => {
      assert(Math.abs(branch[field] - expected[branchIndex][field]) < 1e-11,
        `${fileName}: branch ${branchIndex + 1} ${field}`);
    });
    assert(Math.abs(expression.evaluate(document.expressions[branchIndex].rodLength)
      - branch.rodLength) < 1e-10, `${fileName}: expression branch ${branchIndex + 1} rodLength`);
  });
});

const arbitraryGeometry = model.arbitraryAxisGeometry();
const arbitraryExpressions = model.arbitraryAxisExpressions();
const vectorFields = ["axisPoint", "axis", "crankZero", "platformOffset", "platformPoint"];
arbitraryGeometry.forEach((branch, branchIndex) => {
  vectorFields.forEach((field) => branch[field].forEach((value, component) => {
    const evaluated = expression.evaluate(arbitraryExpressions[branchIndex][field][component]);
    assert(Math.abs(evaluated - value) < 1e-11,
      `arbitrary formula mismatch: branch ${branchIndex + 1} ${field}[${component}]`);
  }));
  assert(Math.abs(expression.evaluate(arbitraryExpressions[branchIndex].rodLength)
    - branch.rodLength) < 1e-12, `arbitrary rod length formula mismatch: branch ${branchIndex + 1}`);
});

const commonOriginGeometry = model.configurationOneGeometry().map((branch) => ({
  ...model.cloneBranch(branch),
  axisPoint: branch.axisPoint.map((value, component) => value - branch.platformOffset[component]),
  platformOffset: [0, 0, 0],
}));
validate("common universal-joint origin geometry", commonOriginGeometry);

const configurationOneExpressions = model.configurationOneExpressions();
model.configurationOneGeometry().forEach((branch, branchIndex) => {
  vectorFields.forEach((field) => branch[field].forEach((value, component) => {
    const evaluated = expression.evaluate(configurationOneExpressions[branchIndex][field][component]);
    assert(Math.abs(evaluated - value) < 1e-12,
      `configuration I formula mismatch: branch ${branchIndex + 1} ${field}[${component}]`);
  }));
});

const roundTrip = model.normalizeGeometry(
  JSON.parse(JSON.stringify(model.configurationOneGeometry())),
);
validate("saved configuration JSON round trip", roundTrip);

let invalidRejected = false;
try {
  const invalid = model.configurationOneGeometry();
  invalid[0].rodLength = 0;
  model.normalizeGeometry(invalid);
} catch (error) {
  invalidRejected = /连杆长度/.test(String(error));
}
assert(invalidRejected, "invalid saved configuration must be rejected");

assert(Math.abs(expression.evaluate("50*cos(15°)") - 50 * Math.cos(15 * Math.PI / 180)) < 1e-12,
  "degree-marked cosine expression");
assert(Math.abs(expression.evaluate("50*cos(15)") - 50 * Math.cos(15 * Math.PI / 180)) < 1e-12,
  "plain-number cosine uses degrees");
assert(Math.abs(expression.evaluate("50*sin(pi/12)") - 50 * Math.sin(Math.PI / 12)) < 1e-12,
  "pi sine expression");
assert(Math.abs(expression.evaluate("sqrt(2)/2") - Math.SQRT1_2) < 1e-12,
  "square-root expression");
assert(expression.evaluate("15°", { angleField: true }) === 15,
  "angle field degree input");
assert(Math.abs(expression.evaluate("pi/12", { angleField: true }) - 15) < 1e-12,
  "angle field pi expression");
let invalidExpressionRejected = false;
try {
  expression.evaluate("sqrt(-1)");
} catch (error) {
  invalidExpressionRejected = /sqrt/.test(String(error));
}
assert(invalidExpressionRejected, "invalid expression must be rejected");
