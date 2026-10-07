// Configure: the blueprint's saved config as typed inputs, in a modal over the view. The blueprint's recommended
// settings first (robots.json's `recommended_config`, from GET /dimos/robots: one config value each, or a pick whose
// choices each set several; an enum shows as buttons for up to 4 choices, else a select; an entry with `when` shows
// only while those values hold; Desktop's Launcher draws the same), then each module's fields (GET/PUT
// /dimos/blueprints/{name}/config), then dimos's global config (GET/PUT /dimos/global-config). Each change is saved at
// once and applies to every later launch of this blueprint (Relaunch to apply it to the running one).

/** a secret as the gateway shows it back: saved, but never sent to a page */
export const HIDDEN = "•••"

export const same = (a, b) => JSON.stringify(a ?? null) === JSON.stringify(b ?? null)

function resolveSchema(schema, root) {
    if (schema.$ref) {
        const target = root.$defs?.[schema.$ref.split("/").pop()] ?? root.definitions?.[schema.$ref.split("/").pop()]
        if (target) {
            return resolveSchema({ ...target, ...schema, $ref: undefined }, root)
        }
    }
    const variants = schema.anyOf ?? schema.oneOf
    if (variants && !schema.type) {
        const nonNull = variants.filter((variant) => variant.type !== "null")
        if (nonNull.length === 1) {
            return { ...resolveSchema(nonNull[0], root), description: schema.description, default: schema.default }
        }
    }
    return schema
}

const nullableSchema = (schema) =>
    (schema.anyOf ?? schema.oneOf ?? []).some((variant) => variant.type === "null") ||
    (Array.isArray(schema.type) && schema.type.includes("null"))

function kindOfSchema(schema) {
    if (schema.enum) {
        return "enum"
    }
    const type = Array.isArray(schema.type) ? schema.type.find((name) => name !== "null") : schema.type
    if (type === "boolean") {
        return "bool"
    }
    if (type === "integer" || type === "number") {
        return type
    }
    if (type === "string") {
        return schema.format === "path" ? "path" : "string"
    }
    return "json"
}

/** a module field's input kind from its Python annotation ("float", "str | None", "pathlib.Path"…) */
function kindOfAnnotation(type) {
    const parts = (type ?? "").split("|").map((part) => part.trim()).filter(Boolean)
    const nullable = parts.includes("None") || /^Optional\[/.test(type ?? "")
    const main = parts.filter((part) => part !== "None")
    if (main.length !== 1) {
        return { kind: "json", nullable }
    }
    const one = main[0].replace(/^Optional\[(.*)\]$/, "$1")
    const kinds = { bool: "bool", int: "integer", float: "number", str: "string" }
    if (kinds[one]) {
        return { kind: kinds[one], nullable }
    }
    return { kind: /(^|\.)Path$/.test(one) ? "path" : "json", nullable }
}

/** a field whose value can't go to or from JSON (a callable, an object) can't be set from here */
function settable(arg) {
    if (arg.json_compatible !== undefined) {
        return arg.json_compatible
    }
    const type = arg.type ?? ""
    return kindOfAnnotation(type).kind !== "json" || !!arg.choices ||
        /^(list|dict|tuple)\[(str|int|float|bool|,|\s)*\]$/.test(type)
}

/** GlobalConfig keys grouped by their first word when two or more share it (zenoh_*, rerun_*…) */
function globalGroups(keys) {
    const word = (key) => key.toLowerCase().split("_")[0]
    const counts = new Map()
    for (const key of keys) {
        counts.set(word(key), (counts.get(word(key)) ?? 0) + 1)
    }
    return new Map(keys.map((key) => [key, counts.get(word(key)) > 1 ? word(key) : "general"]))
}

export function globalFields(config) {
    const properties = config.schema?.properties ?? {}
    const keys = [...new Set([...Object.keys(properties), ...Object.keys(config.defaults ?? {})])]
    const groups = globalGroups(keys)
    const order = (key) => `${groups.get(key) === "general" ? "" : groups.get(key)}\u0000${key.toLowerCase()}`
    keys.sort((a, b) => order(a).localeCompare(order(b)))
    return keys.flatMap((key) => {
        const raw = properties[key] ?? {}
        const schema = resolveSchema(raw, config.schema ?? {})
        const kind = kindOfSchema(schema)
        // lists and objects aren't `dimos` flags: set those in dimos's config file
        if (kind === "json") {
            return []
        }
        return [{
            id: key,
            scope: "global",
            name: key,
            kind,
            options: schema.enum,
            nullable: nullableSchema(raw),
            description: schema.description ?? null,
            base: config.defaults?.[key] ?? schema.default ?? null,
            group: groups.get(key) ?? "general",
            secret: config.secrets?.includes(key) ?? false,
        }]
    })
}

export function moduleFields(config) {
    return config.modules.flatMap((module) =>
        module.args.filter((arg) => !arg.base && settable(arg)).map((arg) => {
            const fromSchema = arg.schema ? kindOfSchema(resolveSchema(arg.schema, arg.schema)) : null
            const annotation = kindOfAnnotation(arg.type)
            return {
                id: `${module.module}.${arg.name}`,
                scope: "module",
                module: module.module,
                name: arg.name,
                kind: arg.choices ? "enum" : fromSchema ?? annotation.kind,
                options: arg.choices,
                nullable: arg.schema ? nullableSchema(arg.schema) : annotation.nullable,
                description: arg.description,
                base: "value" in arg ? arg.value : arg.default,
                group: module.module,
                secret: arg.secret ?? false,
            }
        })
    )
}

const asText = (value) => value === null || value === undefined ? "" : typeof value === "string" ? value : JSON.stringify(value)

export const valueText = (value) =>
    value === null || value === undefined ? "—" : value === "" ? '""' : typeof value === "string" ? value : JSON.stringify(value)

/** the typed value an input's text means, or an error in words */
function parseText(spec, text) {
    const trimmed = text.trim()
    if (trimmed === "" && spec.nullable) {
        return { value: null }
    }
    if (spec.kind === "integer" || spec.kind === "number") {
        const number = Number(trimmed)
        if (trimmed === "" || !Number.isFinite(number)) {
            return { error: "a number" }
        }
        if (spec.kind === "integer" && !Number.isInteger(number)) {
            return { error: "a whole number" }
        }
        return { value: number }
    }
    if (spec.kind === "json") {
        try {
            return { value: JSON.parse(text) }
        } catch {
            return { error: "JSON" }
        }
    }
    return { value: text }
}

/** the blueprint's recommended settings from GET /dimos/robots (none if it isn't listed there) */
async function recommendedOf(name, getJson) {
    try {
        const { robots } = await getJson("robots")
        for (const robot of Object.values(robots ?? {})) {
            const entry = robot.blueprints?.[name]
            if (entry) {
                return entry.recommended_config ?? []
            }
        }
    } catch {
        // no robots.json: no recommended settings
    }
    return []
}

/** Opens the Configure modal over the page. `h` builds elements, `getJson`/`send` talk to the gateway; `onSaved`
 * gets `{global, modules, defaults}` each time the saved config is read or saved. */
export function openConfig({ name, h, getJson, send, onSaved, onClose }) {
    let global = null
    let modules = null
    let recommended = []
    let query = ""
    let error = null
    const body = h("div", { class: "cfg-body" })
    const status = h("span", { class: "cfg-sub" })
    const problem = h("div", { class: "cfg-problem", hidden: true })
    const search = h("input", {
        class: "cfg-search",
        type: "search",
        placeholder: "Search config…",
        "aria-label": "search config",
        oninput: (event) => {
            query = event.target.value
            draw()
        },
    })
    const close = () => {
        scrim.remove()
        removeEventListener("keydown", escape, true)
        onClose?.()
    }
    const escape = (event) => {
        if (event.key === "Escape") {
            event.stopPropagation()
            close()
        }
    }
    addEventListener("keydown", escape, true)
    const dialog = h(
        "div",
        { class: "modal cfg", role: "dialog", "aria-label": `Configure ${name}`, "data-bp-config": true },
        h(
            "div",
            { class: "modal-head" },
            h("span", { class: "label" }, "Config"),
            status,
            h("span", { class: "spacer" }),
            h("button", { type: "button", class: "btn", onclick: close }, "Done"),
        ),
        search,
        h("p", { class: "hint" }, "Saved for every launch of this blueprint. Relaunch to apply."),
        problem,
        body,
    )
    const scrim = h("div", { class: "scrim", onpointerdown: (event) => event.target === scrim && close() }, dialog)
    document.body.append(scrim)
    search.focus()

    const savedGlobal = () => global?.overrides ?? {}
    const savedModules = () => modules?.overrides ?? {}
    const report = () => {
        if (global && modules) {
            onSaved?.({ global: savedGlobal(), modules: savedModules(), defaults: global.defaults ?? {} })
        }
    }
    const fail = (caught) => {
        problem.hidden = false
        problem.textContent = String(caught?.message ?? caught)
    }
    const save = async (action) => {
        problem.hidden = true
        try {
            await action()
            report()
            draw()
        } catch (caught) {
            fail(caught)
        }
    }
    const withModule = (spec, value) => withModuleIn(savedModules(), spec, value)
    const withModuleIn = (overrides, spec, value) => {
        const next = JSON.parse(JSON.stringify(overrides))
        next[spec.module] ??= {}
        if (value === undefined) {
            delete next[spec.module][spec.name]
        } else {
            next[spec.module][spec.name] = value
        }
        if (Object.keys(next[spec.module]).length === 0) {
            delete next[spec.module]
        }
        return next
    }
    const saveModules = async (overrides) => {
        modules = await send("PUT", `blueprints/${encodeURIComponent(name)}/config`, { overrides })
    }
    const saveGlobal = async (overrides) => {
        global = await send("PUT", "global-config", { overrides })
    }
    const setField = (spec, value) =>
        save(() => {
            if (spec.scope === "module") {
                return saveModules(withModule(spec, same(value, spec.base) ? undefined : value))
            }
            const next = { ...savedGlobal() }
            if (same(value, spec.base)) {
                delete next[spec.id]
            } else {
                next[spec.id] = value
            }
            return saveGlobal(next)
        })
    const resetField = (spec) =>
        save(() => {
            if (spec.scope === "module") {
                return saveModules(withModule(spec, undefined))
            }
            const { [spec.id]: _removed, ...rest } = savedGlobal()
            return saveGlobal(rest)
        })
    const valueOf = (spec) =>
        spec.scope === "module"
            ? savedModules()[spec.module]?.[spec.name] ?? spec.base
            : spec.id in savedGlobal()
            ? savedGlobal()[spec.id]
            : spec.base
    const isSaved = (spec) =>
        spec.scope === "module" ? spec.name in (savedModules()[spec.module] ?? {}) : spec.id in savedGlobal()

    function field(spec, compact) {
        const value = valueOf(spec)
        const set = isSaved(spec)
        const bad = h("span", { class: "cfg-bad", hidden: true })
        let input
        if (spec.kind === "bool") {
            input = h("label", { class: "switch" }, h("input", {
                type: "checkbox",
                "aria-label": spec.name,
                onchange: (event) => setField(spec, event.target.checked),
            }), h("span", { class: "track" }))
            input.querySelector("input").checked = value === true
        } else if (spec.kind === "enum") {
            const options = [...(spec.nullable ? [null] : []), ...(spec.options ?? [])]
            input = h(
                "select",
                { "aria-label": spec.name, onchange: (event) => setField(spec, JSON.parse(event.target.value)) },
                options.map((option) =>
                    h(
                        "option",
                        { value: JSON.stringify(option) },
                        spec.optionLabels?.[(spec.options ?? []).indexOf(option)] ??
                            (option === null ? "(none)" : option === "" ? "(empty)" : String(option)),
                    )
                ),
            )
            input.value = JSON.stringify(value ?? null)
        } else {
            const commit = () => {
                if (text.value === asText(value)) {
                    return
                }
                const parsed = parseText(spec, text.value)
                if ("error" in parsed) {
                    bad.hidden = false
                    bad.textContent = `needs ${parsed.error}`
                    return
                }
                setField(spec, parsed.value)
            }
            const text = h("input", {
                class: spec.kind === "string" && !spec.secret ? "" : "mono",
                "aria-label": spec.name,
                type: spec.secret ? "password" : "text",
                autocomplete: spec.secret ? "off" : undefined,
                placeholder: spec.placeholder ?? (spec.nullable ? "(none)" : spec.kind === "path" ? "path" : ""),
                onblur: commit,
                onkeydown: (event) => event.key === "Enter" && commit(),
                onfocus: (event) => spec.secret && event.target.value === HIDDEN && event.target.select(),
            })
            text.value = asText(value)
            input = text
            if (spec.secret) {
                const reveal = h("button", {
                    type: "button",
                    class: "reveal",
                    title: "Saved; never shown again. Type a new one to replace it.",
                    onclick: () => {
                        text.type = text.type === "password" ? "text" : "password"
                        reveal.textContent = text.type === "password" ? "show" : "hide"
                    },
                }, "show")
                input = h("span", { class: "secret" }, text, reveal)
            }
        }
        return h(
            "div",
            { class: `cfg-row${set ? " set" : ""}`, "data-field": spec.id, title: spec.description ?? undefined },
            h("span", { class: "k" }, h("span", { class: "n" }, spec.label ?? spec.name.replaceAll("_", " ")), set && h("span", { class: "mark" }, "saved")),
            h(
                "span",
                { class: "v" },
                input,
                set
                    ? h("button", {
                        type: "button",
                        class: "reset",
                        title: `back to ${valueText(spec.base)}`,
                        "aria-label": `reset ${spec.name}`,
                        onclick: () => resetField(spec),
                    }, "reset")
                    : h("span", { class: "reset-gap" }),
            ),
            bad,
            spec.docs && h("a", { class: "docs", href: spec.docs, target: "_blank", rel: "noopener noreferrer" }, "How do I find this?"),
            !compact && spec.description && h("span", { class: "desc" }, spec.description),
        )
    }

    /** a config value by key (a GlobalConfig field, or <module>.<field>): saved, else its default */
    const keyValue = (key, mFields) => {
        if (key.includes(".")) {
            const [module, fieldName] = [key.slice(0, key.lastIndexOf(".")), key.slice(key.lastIndexOf(".") + 1)]
            const saved = savedModules()[module]
            if (saved && fieldName in saved) {
                return saved[fieldName]
            }
            return mFields.find((spec) => spec.module === module && spec.name === fieldName)?.base
        }
        return key in savedGlobal() ? savedGlobal()[key] : global?.defaults?.[key]
    }
    /** save several values at once (a pick's choice): GlobalConfig fields and module fields */
    const setMany = (values, mFields) =>
        save(async () => {
            const globals = { ...savedGlobal() }
            let moduleNext = savedModules()
            for (const [key, value] of Object.entries(values)) {
                if (key.includes(".")) {
                    const spec = mFields.find((one) => `${one.module}.${one.name}` === key)
                    if (spec) {
                        moduleNext = withModuleIn(moduleNext, spec, same(value, spec.base) ? undefined : value)
                    }
                } else if (same(value, global?.defaults?.[key])) {
                    delete globals[key]
                } else {
                    globals[key] = value
                }
            }
            if (!same(moduleNext, savedModules())) {
                await saveModules(moduleNext)
            }
            if (!same(globals, savedGlobal())) {
                await saveGlobal(globals)
            }
        })
    const shown = (setting, mFields) =>
        !setting.when || Object.entries(setting.when).every(([key, value]) => same(keyValue(key, mFields), value))

    /** buttons for up to 4 options, else a select; `current` is the picked index (-1: none) */
    const choose = (labels, current, pick, name) =>
        labels.length <= 4
            ? h(
                "span",
                { class: "seg", role: "radiogroup", "aria-label": name },
                labels.map((text, index) =>
                    h("button", {
                        type: "button",
                        role: "radio",
                        class: index === current ? "on" : "",
                        "aria-checked": String(index === current),
                        onclick: () => index !== current && pick(index),
                    }, text)
                ),
            )
            : (() => {
                const select = h(
                    "select",
                    { "aria-label": name, onchange: (event) => pick(Number(event.target.value)) },
                    current < 0 && h("option", { value: "-1" }, "—"),
                    labels.map((text, index) => h("option", { value: String(index) }, text)),
                )
                select.value = String(current)
                return select
            })()

    /** one recommended setting as a row: a pick, an enum, or the field it names */
    function recommendedRow(setting, fields, mFields) {
        if (setting.kind === "pick") {
            const current = setting.choices.findIndex((choice) =>
                Object.entries(choice.set ?? {}).every(([key, value]) => same(keyValue(key, mFields), value))
            )
            return h(
                "div",
                { class: "cfg-row pick", "data-field": setting.id },
                h("span", { class: "k" }, h("span", { class: "n" }, setting.label)),
                h("span", { class: "v" }, choose(setting.choices.map((c) => c.label), current, (index) => setMany(setting.choices[index].set, mFields), setting.label)),
            )
        }
        const spec = fields.find((one) => one.id === setting.key && one.scope === setting.scope) ??
            fields.find((one) => setting.scope === "module" && `${one.module}.${one.name}` === setting.key)
        if (!spec) {
            return null
        }
        const labelled = { ...spec, label: setting.label, placeholder: setting.placeholder ?? spec.placeholder, docs: setting.docs }
        if (!setting.choices) {
            return field(labelled, true)
        }
        const value = valueOf(spec)
        const current = setting.choices.findIndex((choice) => same(choice.value, value))
        const set = isSaved(spec)
        return h(
            "div",
            { class: `cfg-row${set ? " set" : ""}`, "data-field": setting.id },
            h("span", { class: "k" }, h("span", { class: "n" }, setting.label), set && h("span", { class: "mark" }, "saved")),
            h(
                "span",
                { class: "v" },
                choose(setting.choices.map((c) => c.label), current, (index) => setField(spec, setting.choices[index].value), setting.label),
                set
                    ? h("button", { type: "button", class: "reset", title: `back to ${valueText(spec.base)}`, onclick: () => resetField(spec) }, "reset")
                    : h("span", { class: "reset-gap" }),
            ),
            setting.docs && h("a", { class: "docs", href: setting.docs, target: "_blank", rel: "noopener noreferrer" }, "How do I find this?"),
        )
    }

    function list(fields, heading) {
        const needle = query.trim().toLowerCase().replaceAll(" ", "_")
        const shown = fields.filter((spec) =>
            !needle || spec.id.toLowerCase().includes(needle) ||
            (spec.description ?? "").toLowerCase().includes(query.trim().toLowerCase())
        )
        if (shown.length === 0) {
            return h("div", { class: "hint" }, query ? `Nothing matches "${query}".` : "Nothing to set.")
        }
        const groups = []
        for (const spec of shown) {
            if (groups.at(-1)?.id === spec.group) {
                groups.at(-1).fields.push(spec)
            } else {
                groups.push({ id: spec.group, fields: [spec] })
            }
        }
        return groups.map((group) =>
            h(
                "div",
                { class: "cfg-group", "data-group": group.id },
                h("div", { class: "cfg-gh" }, heading(group.id), h("span", { class: "n" }, String(group.fields.filter(isSaved).length || ""))),
                group.fields.map((spec) => field(spec, true)),
            )
        )
    }

    function draw() {
        const gFields = global ? globalFields(global) : []
        const mFields = modules ? moduleFields(modules) : []
        const rRows = global && modules
            ? recommended.filter((setting) => shown(setting, mFields)).map((setting) => recommendedRow(setting, [...mFields, ...gFields], mFields)).filter(Boolean)
            : []
        const count = Object.keys(savedGlobal()).length +
            Object.values(savedModules()).reduce((sum, fields) => sum + Object.keys(fields).length, 0)
        status.textContent = count ? `${count} set` : "dimOS defaults"
        if (error) {
            fail(error)
        }
        body.replaceChildren(...[
            rRows.length > 0 && h(
                "section",
                { class: "cfg-sec recommended" },
                h("div", { class: "cfg-sh" }, "Recommended settings"),
                h("p", { class: "hint" }, "Decide these first: the rest can stay at dimOS's defaults."),
                rRows,
            ),
            h(
                "section",
                { class: "cfg-sec" },
                h("div", { class: "cfg-sh" }, "Module config", h("span", { class: "of" }, name)),
                modules ? list(mFields, (group) => group) : h("div", { class: "hint" }, `reading ${name}'s modules…`),
            ),
            h(
                "section",
                { class: "cfg-sec" },
                h("div", { class: "cfg-sh" }, "Global config", h("span", { class: "of" }, "every blueprint")),
                global
                    ? list(gFields, (group) => group === "general" ? "general" : `${group}_*`)
                    : h("div", { class: "hint" }, "reading dimOS's global config…"),
            ),
        ].filter(Boolean))
    }
    draw()
    Promise.all([
        getJson("global-config").then((value) => (global = value)),
        getJson(`blueprints/${encodeURIComponent(name)}/config`).then((value) => (modules = value)),
        recommendedOf(name, getJson).then((value) => (recommended = value)),
    ]).then(() => {
        report()
        draw()
    }, (caught) => {
        error = caught
        draw()
    })
}
