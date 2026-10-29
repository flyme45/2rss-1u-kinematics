(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.Rss1uModel = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const COEFFICIENT_TOLERANCE = 1e-10;
  const JACOBIAN_TOLERANCE = 1e-10;
  const ACTUATOR_RESIDUAL_TOLERANCE = 1e-9;
  const CLOSURE_TOLERANCE = 1e-8;
  const MOTOR_LIMIT = 2.6;

  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
  const norm = (a) => Math.hypot(a[0], a[1], a[2]);

  function unit(a) {
    const length = norm(a);
    if (!Number.isFinite(length) || length < 1e-12) {
      throw new Error("方向向量长度为零");
    }
    return scale(a, 1 / length);
  }

  function matVec(matrix, vector) {
    return matrix.map((row) => dot(row, vector));
  }

  function platformRotation(q) {
    const [pitch, roll] = q;
    const cp = Math.cos(pitch);
    const sp = Math.sin(pitch);
    const cr = Math.cos(roll);
    const sr = Math.sin(roll);
    return [
      [cp, sp * sr, sp * cr],
      [0, cr, -sr],
      [-sp, cp * sr, cp * cr],
    ];
  }

  function rotateAroundAxis(vector, rawAxis, angle) {
    const axis = unit(rawAxis);
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    return add(
      add(scale(vector, cosine), scale(cross(axis, vector), sine)),
      scale(axis, dot(axis, vector) * (1 - cosine)),
    );
  }

  function wrapPi(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  function near(angle, reference) {
    return reference + wrapPi(angle - reference);
  }

  function cloneBranch(branch) {
    return {
      axisPoint: [...branch.axisPoint],
      axis: [...branch.axis],
      crankZero: [...branch.crankZero],
      platformOffset: [...branch.platformOffset],
      platformPoint: [...branch.platformPoint],
      rodLength: branch.rodLength,
      motorOffset: branch.motorOffset || 0,
      motorLower: branch.motorLower ?? -MOTOR_LIMIT,
      motorUpper: branch.motorUpper ?? MOTOR_LIMIT,
    };
  }

  function normalizeGeometry(branches) {
    if (!Array.isArray(branches) || branches.length !== 2) {
      throw new Error("构型必须包含两条 RSS 支链");
    }
    const vectorFields = ["axisPoint", "axis", "crankZero", "platformOffset", "platformPoint"];
    const fieldNames = {
      axisPoint: "电机轴上的点 mᵢ",
      axis: "电机轴方向 uᵢ",
      crankZero: "曲柄零位向量 aᵢ₀",
      platformOffset: "公共万向副中心 O",
      platformPoint: "平台球副向量 bᵢ₀",
    };
    return branches.map((source, index) => {
      const branchNumber = index + 1;
      if (!source || typeof source !== "object") {
        throw new Error(`支链 ${branchNumber} 参数不存在`);
      }
      vectorFields.forEach((field) => {
        if (!Array.isArray(source[field]) || source[field].length !== 3
          || source[field].some((value) => !Number.isFinite(Number(value)))) {
          throw new Error(`支链 ${branchNumber}：${fieldNames[field]}必须包含 x、y、z 三个有效数值`);
        }
      });
      const result = cloneBranch({
        ...source,
        axisPoint: source.axisPoint.map(Number),
        axis: source.axis.map(Number),
        crankZero: source.crankZero.map(Number),
        platformOffset: source.platformOffset.map(Number),
        platformPoint: source.platformPoint.map(Number),
        rodLength: Number(source.rodLength),
        motorOffset: Number(source.motorOffset ?? 0),
        motorLower: Number(source.motorLower ?? -MOTOR_LIMIT),
        motorUpper: Number(source.motorUpper ?? MOTOR_LIMIT),
      });
      if (!Number.isFinite(result.rodLength) || result.rodLength <= 0) {
        throw new Error(`支链 ${branchNumber}：连杆长度 Lᵢ 必须大于 0 mm`);
      }
      if (![result.motorOffset, result.motorLower, result.motorUpper].every(Number.isFinite)) {
        throw new Error(`支链 ${branchNumber} 的电机角参数必须是有限数值`);
      }
      if (result.motorLower > result.motorUpper) {
        throw new Error(`支链 ${branchNumber} 的电机下限不能大于上限`);
      }
      result.axis = unit(result.axis);
      const transverse = sub(result.crankZero, scale(result.axis, dot(result.axis, result.crankZero)));
      if (norm(transverse) < 1e-9) {
        throw new Error(`支链 ${branchNumber}：曲柄零位向量 aᵢ₀ 不能与电机轴方向 uᵢ 平行`);
      }
      return result;
    });
  }

  function currentGeometry() {
    const a1 = (15 * Math.PI) / 180;
    const a2 = (15.0034005 * Math.PI) / 180;
    const a3 = (4.17330971 * Math.PI) / 180;
    const cx = -50 * Math.cos(a1);
    const cz = -50 * Math.sin(a1);
    const bx = -50 * Math.cos(a2);
    const bz = -50 * Math.sin(a2);
    const ax = 315.22 * Math.sin(a3);
    const az = -315.22 * Math.cos(a3);
    return [
      {
        axisPoint: [0, 0, 0],
        axis: [0, 1, 0],
        crankZero: [cx, 0, cz],
        platformOffset: [0, -35.5, -400],
        platformPoint: [cx, 35.5, cz],
        rodLength: 400,
        motorOffset: 0,
        motorLower: -MOTOR_LIMIT,
        motorUpper: MOTOR_LIMIT,
      },
      {
        axisPoint: [0, 0, 0],
        axis: [0, 1, 0],
        crankZero: [bx, 0, bz],
        platformOffset: [ax, 35.5, az],
        platformPoint: [bx, -35.5, bz],
        rodLength: 315.22,
        motorOffset: 0,
        motorLower: -MOTOR_LIMIT,
        motorUpper: MOTOR_LIMIT,
      },
    ];
  }

  function arbitraryAxisGeometry() {
    // 默认“任意空间驱动轴”参数来自用户导出的“空间构型.json”。
    return [
      {
        axisPoint: [-10, 30, 380],
        axis: [-0.49999999999999994, 0.8660254037844387, 0],
        crankZero: [-42.60289043074065, -24.999999999999996, 7.745561757955797],
        platformOffset: [0, 0, 0],
        platformPoint: [-90, 30, -10],
        rodLength: 400.2812457538911,
        motorOffset: 0,
        motorLower: -2.5999999999997554,
        motorUpper: 2.5999999999997554,
      },
      {
        axisPoint: [-10, -30, 330],
        axis: [-0.49999999999999994, -0.8660254037844387, 0],
        crankZero: [-42.65861100064014, 24.999999999999996, 7.432557264903163],
        platformOffset: [0, 0, 0],
        platformPoint: [-90, -30, -10],
        rodLength: 350.3266492575632,
        motorOffset: 0,
        motorLower: -2.5999999999997554,
        motorUpper: 2.5999999999997554,
      },
    ];
  }

  function arbitraryAxisExpressions() {
    return [
      {
        axisPoint: ["-10", "30", "380"],
        axis: ["-sin(30)", "cos(30)", "0"],
        crankZero: [
          "-50*cos(30)*cos(10.30430188)",
          "-50*sin(30)",
          "50*cos(30)*sin(10.30430188)",
        ],
        platformOffset: ["0", "0", "0"],
        platformPoint: ["-90", "30", "-10"],
        rodLength: "400.2812457538911",
        motorOffsetDegrees: "0",
        motorLowerDegrees: "-148.969026734",
        motorUpperDegrees: "148.969026734",
      },
      {
        axisPoint: ["-10", "-30", "330"],
        axis: ["-sin(30)", "-cos(30)", "0"],
        crankZero: [
          "-50*cos(30)*cos(9.88362536)",
          "50*sin(30)",
          "50*cos(30)*sin(9.88362536)",
        ],
        platformOffset: ["0", "0", "0"],
        platformPoint: ["-90", "-30", "-10"],
        rodLength: "350.3266492575632",
        motorOffsetDegrees: "0",
        motorLowerDegrees: "-148.969026734",
        motorUpperDegrees: "148.969026734",
      },
    ];
  }

  function configurationOneGeometry() {
    // 默认“构型 I：固定 x 轴”参数来自用户导出的“x轴.json”。
    return [
      {
        axisPoint: [-41, 0, 195],
        axis: [1, 0, 0],
        crankZero: [0, 23.326823374855284, 8.992180560815337],
        platformOffset: [0, 0, 0],
        platformPoint: [-41, 25, 8.9],
        rodLength: 195.09935529363676,
        motorOffset: 0,
        motorLower: -2.5999999999997554,
        motorUpper: 2.5999999999997554,
      },
      {
        axisPoint: [-41, 0, 140],
        axis: [1, 0, 0],
        crankZero: [0, -23.328001980749068, 8.98912251480463],
        platformOffset: [0, 0, 0],
        platformPoint: [-41, -25, 8.9],
        rodLength: 140.0991000126136,
        motorOffset: 0,
        motorLower: -2.5999999999997554,
        motorUpper: 2.5999999999997554,
      },
    ];
  }

  function configurationOneExpressions() {
    return [
      {
        axisPoint: ["-41", "0", "195"],
        axis: ["1", "0", "0"],
        crankZero: ["0", "25*cos(21.08098853)", "25*sin(21.08098853)"],
        platformOffset: ["0", "0", "0"],
        platformPoint: ["-41", "25", "8.9"],
        rodLength: "195.09935529363676",
        motorOffsetDegrees: "0",
        motorLowerDegrees: "-148.969026734",
        motorUpperDegrees: "148.969026734",
      },
      {
        axisPoint: ["-41", "0", "140"],
        axis: ["1", "0", "0"],
        crankZero: ["0", "-25*cos(21.07347749)", "25*sin(21.07347749)"],
        platformOffset: ["0", "0", "0"],
        platformPoint: ["-41", "-25", "8.9"],
        rodLength: "140.0991000126136",
        motorOffsetDegrees: "0",
        motorLowerDegrees: "-148.969026734",
        motorUpperDegrees: "148.969026734",
      },
    ];
  }

  function movingPoint(branch, q) {
    return add(branch.platformOffset, matVec(platformRotation(q), branch.platformPoint));
  }

  function crankPoint(branch, theta) {
    return add(
      branch.axisPoint,
      rotateAroundAxis(branch.crankZero, branch.axis, theta + (branch.motorOffset || 0)),
    );
  }

  function solveBranch(branch, q, reference) {
    const axis = unit(branch.axis);
    const displacement = sub(movingPoint(branch, q), branch.axisPoint);
    const parallel = scale(axis, dot(axis, branch.crankZero));
    const perpendicular = sub(branch.crankZero, parallel);
    const sineVector = cross(axis, branch.crankZero);
    const K = -2 * dot(displacement, perpendicular);
    const M = -2 * dot(displacement, sineVector);
    const N = dot(displacement, displacement)
      + dot(branch.crankZero, branch.crankZero)
      - branch.rodLength * branch.rodLength
      - 2 * dot(displacement, parallel);
    const amplitude = Math.hypot(K, M);

    if (!Number.isFinite(amplitude) || amplitude < COEFFICIENT_TOLERANCE) {
      throw new Error("K/M 系数退化");
    }
    const normalized = -N / amplitude;
    if (normalized < -1 - COEFFICIENT_TOLERANCE || normalized > 1 + COEFFICIENT_TOLERANCE) {
      throw new Error("目标姿态不可达");
    }
    const phase = Math.atan2(M, K);
    const opening = Math.acos(Math.max(-1, Math.min(1, normalized)));
    const roots = [
      near(phase + opening - (branch.motorOffset || 0), reference),
      near(phase - opening - (branch.motorOffset || 0), reference),
    ];
    const withinLimits = roots.map((value) => value >= branch.motorLower - 1e-10
      && value <= branch.motorUpper + 1e-10);
    return { roots, withinLimits, K, M, N, amplitude };
  }

  function inverse(branches, q, reference) {
    const theta = [0, 0];
    const selectedRoot = [-1, -1];
    const details = [];
    branches.forEach((branch, index) => {
      const solution = solveBranch(branch, q, reference[index]);
      details.push(solution);
      let bestDistance = Infinity;
      solution.roots.forEach((candidate, rootIndex) => {
        if (!solution.withinLimits[rootIndex]) return;
        const distance = Math.abs(candidate - reference[index]);
        if (distance < bestDistance) {
          bestDistance = distance;
          theta[index] = candidate;
          selectedRoot[index] = rootIndex;
        }
      });
      if (selectedRoot[index] < 0) throw new Error(`支链 ${index + 1} 的两个解均超出电机限位`);
    });
    return { theta, selectedRoot, details };
  }

  function closureError(branches, q, theta) {
    return branches.map((branch, index) => (
      norm(sub(movingPoint(branch, q), crankPoint(branch, theta[index]))) - branch.rodLength
    ));
  }

  function invert2(matrix) {
    const determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if (!Number.isFinite(determinant) || Math.abs(determinant) < JACOBIAN_TOLERANCE) {
      throw new Error("2×2 Jacobian 奇异");
    }
    return {
      determinant,
      inverse: [
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
      ],
    };
  }

  function jacobian(branches, q, theta) {
    const pitch = q[0];
    const rotation = platformRotation(q);
    const pitchAxis = [0, 1, 0];
    const rollAxis = [Math.cos(pitch), 0, -Math.sin(pitch)];
    const matrix = [[0, 0], [0, 0]];
    const denominator = [0, 0];

    branches.forEach((branch, index) => {
      const platformVector = matVec(rotation, branch.platformPoint);
      const moving = add(branch.platformOffset, platformVector);
      const crank = crankPoint(branch, theta[index]);
      const rod = sub(moving, crank);
      const crankVelocity = cross(unit(branch.axis), sub(crank, branch.axisPoint));
      denominator[index] = dot(rod, crankVelocity);
      const denominatorScale = Math.max(1, norm(rod) * norm(crankVelocity));
      if (Math.abs(denominator[index]) < JACOBIAN_TOLERANCE * denominatorScale) {
        throw new Error(`支链 ${index + 1} 的 Jacobian 分母退化`);
      }
      matrix[index][0] = dot(rod, cross(pitchAxis, platformVector)) / denominator[index];
      matrix[index][1] = dot(rod, cross(rollAxis, platformVector)) / denominator[index];
    });

    const inverseResult = invert2(matrix);
    const frobenius = Math.hypot(matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]);
    const inverseFrobenius = Math.hypot(
      inverseResult.inverse[0][0], inverseResult.inverse[0][1],
      inverseResult.inverse[1][0], inverseResult.inverse[1][1],
    );
    return {
      matrix,
      denominator,
      determinant: inverseResult.determinant,
      condition: frobenius * inverseFrobenius,
    };
  }

  function multiply2(matrix, vector) {
    return [
      matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
      matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    ];
  }

  function forward(branches, theta, seed) {
    let q = [...seed];
    for (let iteration = 0; iteration < 12; iteration += 1) {
      const predicted = inverse(branches, q, theta);
      const residual = sub2(predicted.theta, theta);
      const differential = jacobian(branches, q, predicted.theta);
      const closure = closureError(branches, q, theta);
      const actuatorConverged = Math.max(
        Math.abs(residual[0]), Math.abs(residual[1]),
      ) <= ACTUATOR_RESIDUAL_TOLERANCE;
      const closureConverged = Math.hypot(...closure) <= CLOSURE_TOLERANCE;
      if (actuatorConverged && closureConverged) {
        return { q, iterations: iteration + 1 };
      }
      const step = multiply2(invert2(differential.matrix).inverse, residual);
      const maximum = Math.max(Math.abs(step[0]), Math.abs(step[1]));
      const stepScale = maximum > 0.2 ? 0.2 / maximum : 1;
      q = [q[0] - stepScale * step[0], q[1] - stepScale * step[1]];
    }
    throw new Error("正运动学迭代达到上限");
  }

  function sub2(a, b) {
    return [a[0] - b[0], a[1] - b[1]];
  }

  function axisFromAngles(azimuth, elevation) {
    const ce = Math.cos(elevation);
    return [ce * Math.cos(azimuth), ce * Math.sin(azimuth), Math.sin(elevation)];
  }

  function axisAngles(rawAxis) {
    const axis = unit(rawAxis);
    return [Math.atan2(axis[1], axis[0]), Math.asin(Math.max(-1, Math.min(1, axis[2])))];
  }

  function reorientBranch(base, rawNewAxis) {
    const result = cloneBranch(base);
    const oldAxis = unit(base.axis);
    const newAxis = unit(rawNewAxis);
    const neutralCrankPoint = add(base.axisPoint, base.crankZero);
    const axialShift = dot(oldAxis, base.crankZero);
    const oldTransverse = sub(base.crankZero, scale(oldAxis, axialShift));
    const radius = norm(oldTransverse);
    let projected = sub(oldTransverse, scale(newAxis, dot(newAxis, oldTransverse)));
    if (norm(projected) < 1e-9) {
      const helper = Math.abs(newAxis[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
      projected = cross(helper, newAxis);
    }
    const transverse = scale(unit(projected), radius);
    result.axis = newAxis;
    result.crankZero = add(transverse, scale(newAxis, axialShift));
    result.axisPoint = sub(neutralCrankPoint, result.crankZero);
    return result;
  }

  function displayGeometry(branches, q, theta) {
    const rotation = platformRotation(q);
    return branches.map((branch, index) => ({
      M: sub(branch.axisPoint, branch.platformOffset),
      A: sub(crankPoint(branch, theta[index]), branch.platformOffset),
      B: matVec(rotation, branch.platformPoint),
      axis: unit(branch.axis),
    }));
  }

  return {
    add, sub, scale, dot, cross, norm, unit, matVec,
    platformRotation, rotateAroundAxis, currentGeometry, arbitraryAxisGeometry,
    arbitraryAxisExpressions,
    configurationOneGeometry, configurationOneExpressions,
    movingPoint, crankPoint, solveBranch, inverse, closureError, jacobian, forward,
    axisFromAngles, axisAngles, reorientBranch, displayGeometry, cloneBranch,
    normalizeGeometry,
  };
});
