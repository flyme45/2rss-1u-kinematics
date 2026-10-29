(function () {
  "use strict";

  const model = window.Rss1uModel;
  const expression = window.Rss1uExpression;
  const DEG = Math.PI / 180;
  const CONFIG_STORAGE_KEY = "rss1u.visualizer.configurations.v1";
  const BUILTIN_PRESET_LABELS = {
    "configuration-one": "构型 I：固定 x 轴",
    paper: "构型 II：固定 y 轴",
    arbitrary: "任意空间驱动轴",
  };
  const VECTOR_PARAMETERS = [
    ["axisPoint", "电机轴上的点 mᵢ / mm"],
    ["axis", "电机轴方向 uᵢ / 无单位"],
    ["crankZero", "曲柄零位向量 aᵢ₀ / mm"],
    ["platformOffset", "公共万向副中心 O / mm"],
    ["platformPoint", "平台球副向量 bᵢ₀ / mm"],
  ];
  const FIELD_NAMES = {
    axisPoint: "电机轴上的点 mᵢ",
    axis: "电机轴方向 uᵢ",
    crankZero: "曲柄零位向量 aᵢ₀",
    platformOffset: "公共万向副中心 O",
    platformPoint: "平台球副向量 bᵢ₀",
    rodLength: "连杆长度 Lᵢ",
    motorOffsetDegrees: "电机零偏",
    motorLowerDegrees: "电机下限",
    motorUpperDegrees: "电机上限",
  };
  const VECTOR_FIELDS = VECTOR_PARAMETERS.map(([field]) => field);
  const SCALAR_FIELDS = [
    "rodLength", "motorOffsetDegrees", "motorLowerDegrees", "motorUpperDegrees",
  ];
  const ANGLE_FIELDS = new Set([
    "motorOffsetDegrees", "motorLowerDegrees", "motorUpperDegrees",
  ]);
  const COLORS = {
    branch: ["#ffad5a", "#55d6e7"],
    motor: "#4f83ff",
    platform: "#e2edf2",
    point: "#f4fbff",
    error: "#ff6f7d",
    grid: "#203846",
  };

  const ui = Object.fromEntries([
    "mechanism-canvas", "preset", "pitch", "roll", "pitch-value", "roll-value",
    "axis1-azimuth", "axis1-elevation", "axis2-azimuth", "axis2-elevation",
    "axis1-azimuth-value", "axis1-elevation-value", "axis2-azimuth-value", "axis2-elevation-value",
    "reset-all", "reset-view", "animate", "header-status-dot", "header-status", "solver-badge",
    "theta1", "theta2", "det-j", "cond-j", "closure-error", "jacobian-matrix",
    "kmn-values", "solver-message", "config-editor", "config-modal", "config-close",
    "config-name", "config-new-copy", "config-branches", "config-status", "config-import",
    "config-export", "config-delete", "config-cancel", "config-save", "config-import-file",
  ].map((id) => [id, document.getElementById(id)]));

  const state = {
    preset: "arbitrary",
    baseBranches: model.arbitraryAxisGeometry(),
    branches: [],
    q: [0, 0],
    theta: [0, 0],
    reference: [0, 0],
    solution: null,
    jacobian: null,
    error: null,
    camera: { yaw: -0.62, pitch: 0.46, zoom: 1 },
    drag: null,
    animating: false,
    animationStart: 0,
    savedConfigurations: [],
    editingConfigurationId: null,
    geometryAdjusted: false,
  };

  function fmt(value, digits = 4) {
    if (!Number.isFinite(value)) return "—";
    if (value !== 0 && (Math.abs(value) >= 1e4 || Math.abs(value) < 1e-3)) {
      return value.toExponential(3);
    }
    return value.toFixed(digits);
  }

  function angleText(radians) {
    const degrees = radians / DEG;
    return `${Math.abs(degrees) < 0.0005 ? "0.000" : degrees.toFixed(3)}°`;
  }

  function cloneGeometry(branches) {
    return branches.map((branch) => model.cloneBranch(branch));
  }

  function normalizeStoredExpressions(source) {
    if (!Array.isArray(source) || source.length !== 2) return null;
    try {
      return source.map((branch) => {
        if (!branch || typeof branch !== "object") throw new Error("表达式格式不正确");
        const normalized = {};
        VECTOR_FIELDS.forEach((field) => {
          if (!Array.isArray(branch[field]) || branch[field].length !== 3) {
            throw new Error("表达式向量格式不正确");
          }
          normalized[field] = branch[field].map((value) => String(value));
        });
        SCALAR_FIELDS.forEach((field) => { normalized[field] = String(branch[field]); });
        return normalized;
      });
    } catch (error) {
      return null;
    }
  }

  function cloneExpressions(source) {
    const normalized = normalizeStoredExpressions(source);
    return normalized ? normalized.map((branch) => ({
      ...branch,
      ...Object.fromEntries(VECTOR_FIELDS.map((field) => [field, [...branch[field]]])),
    })) : null;
  }

  function geometriesClose(left, right) {
    if (!left || !right || left.length !== right.length) return false;
    return left.every((branch, index) => {
      const peer = right[index];
      const vectorsMatch = VECTOR_FIELDS.every((field) => branch[field].every(
        (value, component) => Math.abs(value - peer[field][component]) < 1e-10,
      ));
      return vectorsMatch
        && Math.abs(branch.rodLength - peer.rodLength) < 1e-10
        && Math.abs(branch.motorOffset - peer.motorOffset) < 1e-10
        && Math.abs(branch.motorLower - peer.motorLower) < 1e-10
        && Math.abs(branch.motorUpper - peer.motorUpper) < 1e-10;
    });
  }

  function setConfigurationStatus(message, failed = false) {
    ui["config-status"].textContent = message;
    ui["config-status"].classList.toggle("error", failed);
  }

  function loadSavedConfigurations() {
    try {
      const raw = localStorage.getItem(CONFIG_STORAGE_KEY);
      if (!raw) return;
      const parsed = JSON.parse(raw);
      const items = Array.isArray(parsed) ? parsed : parsed.items;
      if (!Array.isArray(items)) throw new Error("存储格式不正确");
      state.savedConfigurations = items.flatMap((item) => {
        try {
          if (!item || typeof item.name !== "string" || !item.name.trim()) return [];
          return [{
            id: String(item.id || `cfg-${Date.now()}-${Math.random().toString(16).slice(2)}`),
            name: item.name.trim(),
            branches: model.normalizeGeometry(item.branches),
            expressions: normalizeStoredExpressions(item.expressions),
            updatedAt: String(item.updatedAt || ""),
          }];
        } catch (error) {
          return [];
        }
      });
    } catch (error) {
      state.savedConfigurations = [];
      console.warn("读取已保存构型失败：", error);
    }
  }

  function persistSavedConfigurations() {
    localStorage.setItem(CONFIG_STORAGE_KEY, JSON.stringify({
      version: 2,
      items: state.savedConfigurations,
    }));
  }

  function savedPresetValue(id) {
    return `saved:${id}`;
  }

  function savedConfigurationForPreset(value) {
    if (!value.startsWith("saved:")) return null;
    const id = value.slice("saved:".length);
    return state.savedConfigurations.find((item) => item.id === id) || null;
  }

  function refreshSavedPresetOptions(selectedValue = ui.preset.value) {
    document.getElementById("saved-configurations-group")?.remove();
    if (state.savedConfigurations.length) {
      const group = document.createElement("optgroup");
      group.id = "saved-configurations-group";
      group.label = "已保存构型";
      state.savedConfigurations.forEach((item) => {
        const option = document.createElement("option");
        option.value = savedPresetValue(item.id);
        option.textContent = item.name;
        group.append(option);
      });
      ui.preset.append(group);
    }
    const available = [...ui.preset.querySelectorAll("option")]
      .some((option) => option.value === selectedValue);
    ui.preset.value = available ? selectedValue : "configuration-one";
  }

  function updateNumberInputStep(input) {
    const info = expression.plainNumberInfo(input.value);
    const control = input.closest(".expression-control");
    const buttons = control ? [...control.querySelectorAll(".step-button")] : [];
    input.dataset.decimals = info ? String(info.decimals) : "";
    input.dataset.step = info ? String(info.step) : "";
    buttons.forEach((button) => { button.disabled = !info; });
    input.title = info
      ? `上下箭头每次增减 ${info.stepText}`
      : "公式输入：请直接修改表达式";
  }

  function stepNumberInput(input, direction) {
    const info = expression.plainNumberInfo(input.value);
    if (!info) return;
    const factor = 10 ** info.decimals;
    const next = (Math.round(info.value * factor) + direction) / factor;
    input.value = info.decimals ? next.toFixed(info.decimals) : String(next);
    input.dispatchEvent(new Event("input", { bubbles: true }));
  }

  function commonOriginGeometry(branches) {
    const normalized = cloneGeometry(branches);
    const firstOrigin = normalized[0].platformOffset;
    const alreadyShared = normalized.every((branch) => branch.platformOffset.every(
      (value, component) => Math.abs(value - firstOrigin[component]) < 1e-10,
    ));
    if (alreadyShared) return { branches: normalized, converted: false };

    // 旧预设允许每条支链使用自己的局部坐标系。将每条支链同时平移
    // -o_i 后，闭环几何不变，而两条支链可统一在以 O 为原点的坐标系中。
    return {
      branches: normalized.map((branch) => ({
        ...branch,
        axisPoint: branch.axisPoint.map((value, component) => (
          value - branch.platformOffset[component]
        )),
        platformOffset: [0, 0, 0],
      })),
      converted: true,
    };
  }

  function bindSharedPlatformOriginInputs() {
    [0, 1, 2].forEach((component) => {
      const inputs = [...ui["config-branches"].querySelectorAll(
        `[data-field="platformOffset"][data-component="${component}"]`,
      )];
      inputs.forEach((input) => {
        input.addEventListener("input", () => {
          inputs.forEach((peer) => {
            if (peer === input) return;
            peer.value = input.value;
            updateNumberInputStep(peer);
          });
        });
      });
    });
  }

  function createNumberInput(branchIndex, field, value, component = "") {
    const control = document.createElement("div");
    control.className = "expression-control";
    const input = document.createElement("input");
    input.type = "text";
    input.inputMode = "decimal";
    input.autocomplete = "off";
    input.spellcheck = false;
    input.className = "expression-input";
    input.value = String(value);
    input.dataset.branch = String(branchIndex);
    input.dataset.field = field;
    if (component !== "") {
      input.dataset.component = String(component);
      input.setAttribute("aria-label", `支链 ${branchIndex + 1}，${FIELD_NAMES[field]}，${["x", "y", "z"][component]} 分量`);
    } else {
      input.setAttribute("aria-label", `支链 ${branchIndex + 1}，${FIELD_NAMES[field]}`);
    }

    const steppers = document.createElement("span");
    steppers.className = "expression-steppers";
    [[1, "增加"], [-1, "减小"]].forEach(([direction, label]) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "step-button";
      button.textContent = direction > 0 ? "▲" : "▼";
      button.setAttribute("aria-label", `${label}${FIELD_NAMES[field]}`);
      button.addEventListener("click", () => {
        stepNumberInput(input, direction);
        input.focus();
      });
      steppers.append(button);
    });
    control.append(input, steppers);
    updateNumberInputStep(input);
    input.addEventListener("input", () => {
      control.classList.remove("invalid");
      updateNumberInputStep(input);
    });
    input.addEventListener("keydown", (event) => {
      if ((event.key === "ArrowUp" || event.key === "ArrowDown")
          && expression.plainNumberInfo(input.value)) {
        event.preventDefault();
        stepNumberInput(input, event.key === "ArrowUp" ? 1 : -1);
      }
    });
    return control;
  }

  function renderConfigurationForm(branches, storedExpressions = null) {
    const rawExpressions = normalizeStoredExpressions(storedExpressions);
    const rawValue = (branchIndex, field, numericValue, component = null) => {
      const saved = rawExpressions?.[branchIndex]?.[field];
      if (component !== null && Array.isArray(saved)) return saved[component];
      if (component === null && typeof saved === "string") return saved;
      return expression.formatNumber(numericValue);
    };
    ui["config-branches"].replaceChildren();
    branches.forEach((branch, branchIndex) => {
      const fieldset = document.createElement("fieldset");
      fieldset.className = "branch-editor";
      const legend = document.createElement("legend");
      legend.textContent = `支链 ${branchIndex + 1}`;
      fieldset.append(legend);

      const heading = document.createElement("div");
      heading.className = "parameter-row";
      heading.append(document.createElement("span"));
      ["x", "y", "z"].forEach((axis) => {
        const label = document.createElement("span");
        label.className = "vector-heading";
        label.textContent = axis;
        heading.append(label);
      });
      fieldset.append(heading);

      VECTOR_PARAMETERS.forEach(([field, labelText]) => {
        const row = document.createElement("div");
        row.className = `parameter-row${field === "platformOffset" ? " shared-parameter-row" : ""}`;
        const label = document.createElement("label");
        label.textContent = labelText;
        if (field === "platformOffset") {
          const badge = document.createElement("span");
          badge.className = "shared-badge";
          badge.textContent = "两支链自动同步";
          label.append(document.createElement("br"), badge);
        }
        row.append(label);
        branch[field].forEach((value, component) => {
          row.append(createNumberInput(
            branchIndex, field, rawValue(branchIndex, field, value, component), component,
          ));
        });
        fieldset.append(row);
      });

      const scalarGrid = document.createElement("div");
      scalarGrid.className = "scalar-grid";
      [
        ["rodLength", "连杆长度 Lᵢ / mm", branch.rodLength, true],
        ["motorOffsetDegrees", "电机零偏 / °", (branch.motorOffset || 0) / DEG, false],
        ["motorLowerDegrees", "电机下限 / °", branch.motorLower / DEG, false],
        ["motorUpperDegrees", "电机上限 / °", branch.motorUpper / DEG, false],
      ].forEach(([field, labelText, value, wide]) => {
        const wrapper = document.createElement("div");
        wrapper.className = `scalar-field${wide ? " wide" : ""}`;
        const label = document.createElement("label");
        label.textContent = labelText;
        wrapper.append(label, createNumberInput(
          branchIndex, field, rawValue(branchIndex, field, value),
        ));
        scalarGrid.append(wrapper);
      });
      fieldset.append(scalarGrid);
      ui["config-branches"].append(fieldset);
    });
    bindSharedPlatformOriginInputs();
  }

  function readConfigurationForm() {
    ui["config-branches"].querySelectorAll(".expression-control.invalid")
      .forEach((control) => control.classList.remove("invalid"));

    function inputOf(branchIndex, field, component = null) {
      const componentSelector = component === null ? "" : `[data-component="${component}"]`;
      return ui["config-branches"].querySelector(
        `[data-branch="${branchIndex}"][data-field="${field}"]${componentSelector}`,
      );
    }

    function valueOf(branchIndex, field, component = null) {
      const input = inputOf(branchIndex, field, component);
      try {
        if (!input) throw new Error("找不到输入框");
        return expression.evaluate(input.value, { angleField: ANGLE_FIELDS.has(field) });
      } catch (error) {
        input?.closest(".expression-control")?.classList.add("invalid");
        input?.focus();
        const componentText = component === null ? "" : `的 ${["x", "y", "z"][component]} 分量`;
        const detail = error instanceof Error ? error.message : String(error);
        throw new Error(`支链 ${branchIndex + 1}：${FIELD_NAMES[field] || field}${componentText}公式错误：${detail}`);
      }
    }

    const expressionText = (branchIndex, field, component = null) => (
      inputOf(branchIndex, field, component)?.value.trim() || ""
    );

    const sharedPlatformOffset = [0, 1, 2].map((component) => (
      valueOf(0, "platformOffset", component)
    ));
    const branches = [0, 1].map((branchIndex) => {
      const vector = (field) => [0, 1, 2].map((component) => valueOf(branchIndex, field, component));
      return {
        axisPoint: vector("axisPoint"),
        axis: vector("axis"),
        crankZero: vector("crankZero"),
        platformOffset: [...sharedPlatformOffset],
        platformPoint: vector("platformPoint"),
        rodLength: valueOf(branchIndex, "rodLength"),
        motorOffset: valueOf(branchIndex, "motorOffsetDegrees") * DEG,
        motorLower: valueOf(branchIndex, "motorLowerDegrees") * DEG,
        motorUpper: valueOf(branchIndex, "motorUpperDegrees") * DEG,
      };
    });
    const expressions = [0, 1].map((branchIndex) => ({
      ...Object.fromEntries(VECTOR_FIELDS.map((field) => [
        field,
        [0, 1, 2].map((component) => (
          field === "platformOffset"
            ? expressionText(0, field, component)
            : expressionText(branchIndex, field, component)
        )),
      ])),
      ...Object.fromEntries(SCALAR_FIELDS.map((field) => [
        field, expressionText(branchIndex, field),
      ])),
    }));
    return { branches: model.normalizeGeometry(branches), expressions };
  }

  function currentConfigurationName() {
    const saved = savedConfigurationForPreset(state.preset);
    if (saved) return saved.name;
    return `${BUILTIN_PRESET_LABELS[state.preset] || "当前构型"} 副本`;
  }

  function openConfigurationEditor() {
    stopAnimation();
    const saved = savedConfigurationForPreset(state.preset);
    state.editingConfigurationId = saved?.id || null;
    ui["config-name"].value = currentConfigurationName();
    const editingGeometry = saved && !state.geometryAdjusted
      ? state.baseBranches
      : (state.branches.length ? state.branches : state.baseBranches);
    const commonGeometry = commonOriginGeometry(editingGeometry);
    let storedExpressions = null;
    if (saved && !state.geometryAdjusted && !commonGeometry.converted
        && geometriesClose(state.baseBranches, saved.branches)) {
      storedExpressions = saved.expressions;
    } else if (!saved && !state.geometryAdjusted && !commonGeometry.converted) {
      const builtInExpressions = {
        "configuration-one": model.configurationOneExpressions,
        arbitrary: model.arbitraryAxisExpressions,
      };
      storedExpressions = builtInExpressions[state.preset]?.() || null;
    }
    renderConfigurationForm(commonGeometry.branches, storedExpressions);
    ui["config-delete"].disabled = !state.editingConfigurationId;
    setConfigurationStatus(commonGeometry.converted
      ? "已将旧预设的两个支链局部坐标系等效转换为公共 O 坐标系；机构几何保持不变。"
      : "公共万向副中心 O 已联动；修改任意一侧，另一侧会立即同步。");
    ui["config-modal"].hidden = false;
    ui["config-name"].focus();
    ui["config-name"].select();
  }

  function closeConfigurationEditor() {
    ui["config-modal"].hidden = true;
    state.editingConfigurationId = null;
    ui["config-import-file"].value = "";
  }

  function saveConfiguration() {
    try {
      const name = ui["config-name"].value.trim();
      if (!name) throw new Error("请输入构型名称");
      const { branches, expressions } = readConfigurationForm();
      const id = state.editingConfigurationId
        || `cfg-${Date.now()}-${Math.random().toString(16).slice(2)}`;
      const record = {
        id,
        name,
        branches: cloneGeometry(branches),
        expressions: cloneExpressions(expressions),
        updatedAt: new Date().toISOString(),
      };
      const index = state.savedConfigurations.findIndex((item) => item.id === id);
      if (index >= 0) state.savedConfigurations[index] = record;
      else state.savedConfigurations.push(record);
      persistSavedConfigurations();
      const presetValue = savedPresetValue(id);
      refreshSavedPresetOptions(presetValue);
      state.editingConfigurationId = id;
      choosePreset(presetValue);
      closeConfigurationEditor();
    } catch (error) {
      setConfigurationStatus(error instanceof Error ? error.message : String(error), true);
    }
  }

  function exportConfiguration() {
    try {
      const name = ui["config-name"].value.trim() || "RSS1U 构型";
      const { branches, expressions } = readConfigurationForm();
      const content = JSON.stringify({ version: 2, name, branches, expressions }, null, 2);
      const blob = new Blob([content], { type: "application/json;charset=utf-8" });
      const link = document.createElement("a");
      link.href = URL.createObjectURL(blob);
      link.download = `${name.replace(/[\\/:*?"<>|]/g, "_")}.json`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(link.href), 0);
      setConfigurationStatus("当前表单已导出为 JSON。", false);
    } catch (error) {
      setConfigurationStatus(error instanceof Error ? error.message : String(error), true);
    }
  }

  async function importConfigurationFile(file) {
    try {
      const parsed = JSON.parse(await file.text());
      const source = parsed.branches ? parsed : (Array.isArray(parsed.items) ? parsed.items[0] : null);
      if (!source) throw new Error("JSON 中未找到 branches 参数");
      const imported = commonOriginGeometry(model.normalizeGeometry(source.branches));
      const importedExpressions = imported.converted
        ? null
        : normalizeStoredExpressions(source.expressions);
      state.editingConfigurationId = null;
      ui["config-name"].value = String(source.name || file.name.replace(/\.json$/i, "") || "导入构型");
      renderConfigurationForm(imported.branches, importedExpressions);
      ui["config-delete"].disabled = true;
      setConfigurationStatus(imported.converted
        ? "JSON 已载入，并已等效转换为公共 O 坐标系。请检查后保存。"
        : "JSON 已载入。公共万向副中心 O 已联动，请检查后保存。", false);
    } catch (error) {
      setConfigurationStatus(`导入失败：${error instanceof Error ? error.message : String(error)}`, true);
    }
  }

  function deleteConfiguration() {
    if (!state.editingConfigurationId) return;
    const record = state.savedConfigurations.find((item) => item.id === state.editingConfigurationId);
    if (!record || !window.confirm(`确定删除已保存构型“${record.name}”吗？`)) return;
    state.savedConfigurations = state.savedConfigurations.filter(
      (item) => item.id !== state.editingConfigurationId,
    );
    try {
      persistSavedConfigurations();
      refreshSavedPresetOptions("configuration-one");
      choosePreset("configuration-one");
      closeConfigurationEditor();
    } catch (error) {
      setConfigurationStatus(`删除失败：${error instanceof Error ? error.message : String(error)}`, true);
    }
  }

  function getAxisInputs(index) {
    return index === 0
      ? [ui["axis1-azimuth"], ui["axis1-elevation"]]
      : [ui["axis2-azimuth"], ui["axis2-elevation"]];
  }

  function setAxisControlsFromGeometry() {
    state.baseBranches.forEach((branch, index) => {
      const [azimuth, elevation] = model.axisAngles(branch.axis).map((value) => value / DEG);
      const [azimuthInput, elevationInput] = getAxisInputs(index);
      azimuthInput.value = azimuth.toFixed(1);
      elevationInput.value = elevation.toFixed(1);
    });
    updateControlOutputs();
  }

  function updateControlOutputs() {
    ui["pitch-value"].textContent = `${Number(ui.pitch.value).toFixed(1)}°`;
    ui["roll-value"].textContent = `${Number(ui.roll.value).toFixed(1)}°`;
    [1, 2].forEach((number) => {
      ui[`axis${number}-azimuth-value`].textContent = `${Number(ui[`axis${number}-azimuth`].value).toFixed(1)}°`;
      ui[`axis${number}-elevation-value`].textContent = `${Number(ui[`axis${number}-elevation`].value).toFixed(1)}°`;
    });
  }

  function rebuildBranches() {
    state.branches = state.baseBranches.map((branch, index) => {
      const [azimuthInput, elevationInput] = getAxisInputs(index);
      const axis = model.axisFromAngles(Number(azimuthInput.value) * DEG, Number(elevationInput.value) * DEG);
      return model.reorientBranch(branch, axis);
    });
  }

  function updateSolver(resetReference = false) {
    updateControlOutputs();
    state.q = [Number(ui.pitch.value) * DEG, Number(ui.roll.value) * DEG];
    if (resetReference) state.reference = [0, 0];
    rebuildBranches();

    try {
      const solution = model.inverse(state.branches, state.q, state.reference);
      state.theta = [...solution.theta];
      state.reference = [...solution.theta];
      state.solution = solution;
      state.jacobian = model.jacobian(state.branches, state.q, state.theta);
      state.error = null;
    } catch (error) {
      state.solution = null;
      state.jacobian = null;
      state.error = error instanceof Error ? error.message : String(error);
    }
    updateDiagnostics();
    draw();
  }

  function updateDiagnostics() {
    const failed = Boolean(state.error);
    ui["header-status-dot"].classList.toggle("error", failed);
    ui["header-status"].textContent = failed ? "当前姿态不可用" : "求解正常";
    ui["solver-badge"].classList.toggle("error", failed);
    ui["solver-badge"].textContent = failed ? "ERROR" : "OK";
    ui["solver-message"].classList.toggle("error", failed);
    ui["solver-message"].textContent = failed ? state.error : "连续装配分支已保持，闭环约束满足。";

    if (failed) {
      ["theta1", "theta2", "det-j", "cond-j", "closure-error", "jacobian-matrix", "kmn-values"]
        .forEach((id) => { ui[id].textContent = "—"; });
      return;
    }

    const closure = model.closureError(state.branches, state.q, state.theta);
    ui.theta1.textContent = angleText(state.theta[0]);
    ui.theta2.textContent = angleText(state.theta[1]);
    ui["det-j"].textContent = fmt(state.jacobian.determinant);
    ui["cond-j"].textContent = fmt(state.jacobian.condition, 3);
    ui["closure-error"].textContent = `[ ${fmt(closure[0], 6)},  ${fmt(closure[1], 6)} ]`;
    const J = state.jacobian.matrix;
    ui["jacobian-matrix"].textContent = `[ ${fmt(J[0][0])}  ${fmt(J[0][1])} ]\n[ ${fmt(J[1][0])}  ${fmt(J[1][1])} ]`;
    ui["kmn-values"].textContent = state.solution.details.map((detail, index) => (
      `支链 ${index + 1}  K=${fmt(detail.K, 2)}  M=${fmt(detail.M, 2)}\n`
      + `        N=${fmt(detail.N, 2)}  根=${state.solution.selectedRoot[index] + 1}`
    )).join("\n");
  }

  function cameraTransform(point) {
    const cy = Math.cos(state.camera.yaw);
    const sy = Math.sin(state.camera.yaw);
    const cp = Math.cos(state.camera.pitch);
    const sp = Math.sin(state.camera.pitch);
    const x1 = cy * point[0] - sy * point[1];
    const y1 = sy * point[0] + cy * point[1];
    const z1 = point[2];
    return [x1, cp * y1 - sp * z1, sp * y1 + cp * z1];
  }

  function collectScene() {
    const rows = model.displayGeometry(state.branches, state.q, state.theta);
    const segments = [];
    rows.forEach((row, index) => {
      const axisLength = Math.max(65, state.branches[index].rodLength * 0.24);
      const axisStart = model.sub(row.M, model.scale(row.axis, axisLength));
      const axisEnd = model.add(row.M, model.scale(row.axis, axisLength));
      segments.push(
        { a: axisStart, b: axisEnd, color: COLORS.motor, width: 3, kind: "axis" },
        { a: row.M, b: row.A, color: COLORS.branch[index], width: 6, kind: "crank" },
        { a: row.A, b: row.B, color: state.error ? COLORS.error : COLORS.branch[index], width: 4, kind: "rod" },
      );
    });
    segments.push(
      { a: rows[0].B, b: rows[1].B, color: COLORS.platform, width: 5, kind: "platform" },
      { a: [0, 0, 0], b: rows[0].B, color: COLORS.platform, width: 2, kind: "platform" },
      { a: [0, 0, 0], b: rows[1].B, color: COLORS.platform, width: 2, kind: "platform" },
    );
    return { rows, segments };
  }

  function geometryRadius() {
    // 使用与当前姿态无关的几何包络确定比例，避免机构运动时相机自动缩放。
    // B 点绕 O 旋转，因此其包络半径恒为 |platformPoint|；A 点的保守
    // 包络半径为 |M| + |crankZero|。
    let radius = 90;
    state.branches.forEach((branch) => {
      const motorCenter = model.sub(branch.axisPoint, branch.platformOffset);
      const motorAxisLength = Math.max(65, branch.rodLength * 0.24);
      radius = Math.max(
        radius,
        model.norm(branch.platformPoint),
        model.norm(motorCenter) + model.norm(branch.crankZero),
        model.norm(motorCenter) + motorAxisLength,
      );
    });
    return Math.max(radius, 1);
  }

  function setupCanvas() {
    const canvas = ui["mechanism-canvas"];
    const rect = canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width * ratio));
    const height = Math.max(1, Math.round(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const context = canvas.getContext("2d");
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { context, width: rect.width, height: rect.height };
  }

  function drawGrid(context, width, height) {
    context.save();
    context.strokeStyle = "rgba(77, 122, 140, 0.12)";
    context.lineWidth = 1;
    const spacing = 42;
    for (let x = spacing; x < width; x += spacing) {
      context.beginPath(); context.moveTo(x, 0); context.lineTo(x, height); context.stroke();
    }
    for (let y = spacing; y < height; y += spacing) {
      context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke();
    }
    context.restore();
  }

  function draw() {
    const { context, width, height } = setupCanvas();
    context.clearRect(0, 0, width, height);
    drawGrid(context, width, height);
    if (!state.branches.length) return;

    const scene = collectScene();
    const fitScale = Math.min(width, height) * 0.43
      / geometryRadius() * state.camera.zoom;
    // 屏幕锚点始终对应世界坐标原点 O。机构姿态、装配分支和相机旋转
    // 只改变其他点相对 O 的位置，不再触发逐帧重新居中。
    const anchorX = width * 0.5;
    const anchorY = height * 0.5;
    const project = (point) => {
      const view = cameraTransform(point);
      return {
        x: anchorX + view[0] * fitScale,
        y: anchorY - view[2] * fitScale,
        depth: view[1],
      };
    };

    const axisLength = Math.max(...state.branches.map((branch) => branch.rodLength)) * 0.25;
    const worldAxes = [
      { end: [axisLength, 0, 0], color: "#ef6f6c", label: "x" },
      { end: [0, axisLength, 0], color: "#66d28e", label: "y" },
      { end: [0, 0, axisLength], color: "#6a9eff", label: "z" },
    ];
    worldAxes.forEach((axis) => {
      const start = project([0, 0, 0]);
      const end = project(axis.end);
      drawLine(context, start, end, axis.color, 2);
      drawArrow(context, start, end, axis.color, 8);
      drawLabel(context, axis.label, end.x + 7, end.y - 5, axis.color);
    });

    scene.segments
      .map((segment) => ({ ...segment, pa: project(segment.a), pb: project(segment.b) }))
      .sort((a, b) => ((a.pa.depth + a.pb.depth) - (b.pa.depth + b.pb.depth)))
      .forEach((segment) => {
        drawLine(context, segment.pa, segment.pb, segment.color, segment.width);
        if (segment.kind === "axis") drawArrow(context, segment.pa, segment.pb, segment.color, 10);
      });

    const origin = project([0, 0, 0]);
    drawJoint(context, origin, COLORS.platform, 6);
    drawLabel(context, "O", origin.x + 9, origin.y - 8, COLORS.platform);
    scene.rows.forEach((row, index) => {
      [["M", row.M], ["A", row.A], ["B", row.B]].forEach(([name, point]) => {
        const screen = project(point);
        drawJoint(context, screen, name === "M" ? COLORS.motor : COLORS.point, name === "M" ? 7 : 6);
        drawLabel(context, `${name}₍${index + 1}₎`, screen.x + 8, screen.y - 8, COLORS.branch[index]);
      });
      const axisTip = project(model.add(row.M, model.scale(row.axis, axisLength * 0.7)));
      drawLabel(context, `u₍${index + 1}₎`, axisTip.x + 7, axisTip.y + 12, COLORS.motor);
    });

    context.save();
    context.fillStyle = "rgba(148, 181, 194, .8)";
    context.font = "12px Segoe UI, Microsoft YaHei UI, sans-serif";
    context.fillText(`Pitch ${Number(ui.pitch.value).toFixed(1)}°   Roll ${Number(ui.roll.value).toFixed(1)}°`, 18, 28);
    context.restore();
  }

  function drawLine(context, a, b, color, width) {
    context.save();
    context.strokeStyle = color;
    context.lineWidth = width;
    context.lineCap = "round";
    context.shadowColor = color;
    context.shadowBlur = width > 3 ? 5 : 0;
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.stroke();
    context.restore();
  }

  function drawArrow(context, a, b, color, size) {
    const angle = Math.atan2(b.y - a.y, b.x - a.x);
    context.save();
    context.fillStyle = color;
    context.beginPath();
    context.moveTo(b.x, b.y);
    context.lineTo(b.x - size * Math.cos(angle - 0.45), b.y - size * Math.sin(angle - 0.45));
    context.lineTo(b.x - size * Math.cos(angle + 0.45), b.y - size * Math.sin(angle + 0.45));
    context.closePath();
    context.fill();
    context.restore();
  }

  function drawJoint(context, point, color, radius) {
    context.save();
    context.fillStyle = "#071017";
    context.strokeStyle = color;
    context.lineWidth = 2.5;
    context.beginPath();
    context.arc(point.x, point.y, radius, 0, Math.PI * 2);
    context.fill();
    context.stroke();
    context.restore();
  }

  function drawLabel(context, text, x, y, color) {
    context.save();
    context.fillStyle = color;
    context.font = "600 12px Segoe UI, Microsoft YaHei UI, sans-serif";
    context.shadowColor = "#061016";
    context.shadowBlur = 4;
    context.fillText(text, x, y);
    context.restore();
  }

  function choosePreset(name) {
    const presets = {
      "configuration-one": model.configurationOneGeometry,
      paper: model.currentGeometry,
      arbitrary: model.arbitraryAxisGeometry,
    };
    const saved = savedConfigurationForPreset(name);
    if (!presets[name] && !saved) name = "configuration-one";
    state.preset = name;
    state.baseBranches = saved
      ? cloneGeometry(saved.branches)
      : (presets[name] || model.configurationOneGeometry)();
    ui.preset.value = name;
    state.reference = [0, 0];
    state.theta = [0, 0];
    state.geometryAdjusted = false;
    ui.pitch.value = "0";
    ui.roll.value = "0";
    setAxisControlsFromGeometry();
    updateSolver(true);
  }

  function resetView() {
    state.camera = { yaw: -0.62, pitch: 0.46, zoom: 1 };
    draw();
  }

  function stopAnimation() {
    state.animating = false;
    ui.animate.textContent = "开始动画";
  }

  function animationFrame(time) {
    if (!state.animating) return;
    if (!state.animationStart) state.animationStart = time;
    const seconds = (time - state.animationStart) / 1000;
    ui.pitch.value = (14 * Math.sin(seconds * 0.7)).toFixed(2);
    ui.roll.value = (11 * Math.sin(seconds * 0.93 + 0.65)).toFixed(2);
    updateSolver();
    requestAnimationFrame(animationFrame);
  }

  [ui.pitch, ui.roll].forEach((input) => input.addEventListener("input", () => {
    stopAnimation();
    updateSolver();
  }));
  [ui["axis1-azimuth"], ui["axis1-elevation"], ui["axis2-azimuth"], ui["axis2-elevation"]]
    .forEach((input) => input.addEventListener("input", () => {
      stopAnimation();
      state.geometryAdjusted = true;
      updateSolver(true);
    }));
  ui.preset.addEventListener("change", () => choosePreset(ui.preset.value));
  ui["config-editor"].addEventListener("click", openConfigurationEditor);
  ui["config-close"].addEventListener("click", closeConfigurationEditor);
  ui["config-cancel"].addEventListener("click", closeConfigurationEditor);
  ui["config-modal"].querySelector("[data-close-config]")
    .addEventListener("click", closeConfigurationEditor);
  ui["config-new-copy"].addEventListener("click", () => {
    state.editingConfigurationId = null;
    ui["config-name"].value = `${ui["config-name"].value.trim() || "新构型"} 副本`;
    ui["config-delete"].disabled = true;
    setConfigurationStatus("已切换为新构型；保存后会新增到预设列表。", false);
    ui["config-name"].focus();
    ui["config-name"].select();
  });
  ui["config-save"].addEventListener("click", saveConfiguration);
  ui["config-export"].addEventListener("click", exportConfiguration);
  ui["config-import"].addEventListener("click", () => ui["config-import-file"].click());
  ui["config-import-file"].addEventListener("change", () => {
    const [file] = ui["config-import-file"].files;
    if (file) importConfigurationFile(file);
  });
  ui["config-delete"].addEventListener("click", deleteConfiguration);
  ui["reset-all"].addEventListener("click", () => choosePreset(state.preset));
  ui["reset-view"].addEventListener("click", resetView);
  ui.animate.addEventListener("click", () => {
    state.animating = !state.animating;
    ui.animate.textContent = state.animating ? "暂停动画" : "开始动画";
    if (state.animating) {
      state.animationStart = 0;
      requestAnimationFrame(animationFrame);
    }
  });

  const canvas = ui["mechanism-canvas"];
  canvas.addEventListener("pointerdown", (event) => {
    canvas.setPointerCapture(event.pointerId);
    state.drag = { x: event.clientX, y: event.clientY, yaw: state.camera.yaw, pitch: state.camera.pitch };
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!state.drag) return;
    state.camera.yaw = state.drag.yaw + (event.clientX - state.drag.x) * 0.008;
    state.camera.pitch = Math.max(-1.35, Math.min(1.35, state.drag.pitch + (event.clientY - state.drag.y) * 0.008));
    draw();
  });
  canvas.addEventListener("pointerup", () => { state.drag = null; });
  canvas.addEventListener("pointercancel", () => { state.drag = null; });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.camera.zoom = Math.max(0.55, Math.min(2.4, state.camera.zoom * Math.exp(-event.deltaY * 0.001)));
    draw();
  }, { passive: false });
  canvas.addEventListener("dblclick", resetView);

  new ResizeObserver(draw).observe(canvas);
  window.addEventListener("keydown", (event) => {
    if (event.key.toLowerCase() === "r") resetView();
    if (event.key === "Escape" && !ui["config-modal"].hidden) closeConfigurationEditor();
    else if (event.key === "Escape") stopAnimation();
  });

  loadSavedConfigurations();
  refreshSavedPresetOptions("configuration-one");
  choosePreset("configuration-one");
})();
