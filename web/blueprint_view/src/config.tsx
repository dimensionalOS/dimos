// The config dialog: the blueprint's recommended settings, its modules' args and dimos's global config, each saved
// for every launch of this blueprint (PUT /blueprints/{name}/config, PUT /global-config).
import {
  Alert,
  Button,
  Modal,
  sameValue,
  SettingChoice,
  SettingGroup,
  SettingInput,
  SettingRow,
  SettingSection,
  TextInput,
} from "dimos-react";
import { type ReactNode, useEffect, useState } from "react";
import { errorText, getJson, send } from "./api.ts";
import {
  type BlueprintConfig,
  type FieldSpec,
  type GlobalConfig,
  globalFields,
  moduleFields,
} from "./config_fields.ts";

type Overrides = Record<string, unknown>;
type ModuleOverrides = Record<string, Record<string, unknown>>;

export interface Saved {
  global: Overrides;
  modules: ModuleOverrides;
  defaults: Overrides;
}

interface Recommended {
  id: string;
  kind?: string;
  key?: string;
  scope?: string;
  label: string;
  placeholder?: string;
  docs?: string;
  when?: Record<string, unknown>;
  choices?: { label: string; value?: unknown; set?: Record<string, unknown> }[];
}

async function recommendedOf(name: string): Promise<Recommended[]> {
  try {
    const { robots } = await getJson<
      {
        robots?: Record<
          string,
          { blueprints?: Record<string, { recommended_config?: Recommended[] }> }
        >;
      }
    >("robots");
    for (const robot of Object.values(robots ?? {})) {
      const entry = robot.blueprints?.[name];
      if (entry) {
        return entry.recommended_config ?? [];
      }
    }
  } catch {
    // no robots: no recommended settings
  }
  return [];
}

function withModule(overrides: ModuleOverrides, spec: FieldSpec, value: unknown): ModuleOverrides {
  const next: ModuleOverrides = JSON.parse(JSON.stringify(overrides));
  const module = spec.module!;
  next[module] ??= {};
  if (value === undefined) {
    delete next[module][spec.name];
  } else {
    next[module][spec.name] = value;
  }
  if (Object.keys(next[module]).length === 0) {
    delete next[module];
  }
  return next;
}

export function ConfigModal(
  { name, onSaved, onClose }: {
    name: string;
    onSaved: (saved: Saved) => void;
    onClose: () => void;
  },
) {
  const [global, setGlobal] = useState<GlobalConfig | null>(null);
  const [modules, setModules] = useState<BlueprintConfig | null>(null);
  const [recommended, setRecommended] = useState<Recommended[]>([]);
  const [query, setQuery] = useState("");
  const [problem, setProblem] = useState<string | null>(null);

  useEffect(() => {
    Promise.all([
      getJson<GlobalConfig>("global-config"),
      getJson<{ config: BlueprintConfig }>(`blueprints/${encodeURIComponent(name)}`),
      recommendedOf(name),
    ]).then(([g, b, r]) => {
      setGlobal(g);
      setModules(b.config);
      setRecommended(r);
    }, (caught) => setProblem(errorText(caught)));
  }, [name]);

  const savedGlobal = global?.overrides ?? {};
  const savedModules = modules?.overrides ?? {};
  useEffect(() => {
    if (global && modules) {
      onSaved({ global: savedGlobal, modules: savedModules, defaults: global.defaults ?? {} });
    }
    // report what's saved whenever it changes
  }, [global, modules]);

  const save = async (action: () => Promise<void>) => {
    setProblem(null);
    try {
      await action();
    } catch (caught) {
      setProblem(errorText(caught));
    }
  };
  const saveModules = async (overrides: ModuleOverrides) =>
    setModules(
      await send<BlueprintConfig>("PUT", `blueprints/${encodeURIComponent(name)}/config`, {
        overrides,
      }),
    );
  const saveGlobal = async (overrides: Overrides) =>
    setGlobal(await send<GlobalConfig>("PUT", "global-config", { overrides }));
  const setField = (spec: FieldSpec, value: unknown) =>
    save(() => {
      const atBase = sameValue(value, spec.base);
      if (spec.scope === "module") {
        return saveModules(withModule(savedModules, spec, atBase ? undefined : value));
      }
      const { [spec.id]: _old, ...rest } = savedGlobal;
      return saveGlobal(atBase ? rest : { ...rest, [spec.id]: value });
    });
  const resetField = (spec: FieldSpec) => setField(spec, spec.base);
  const valueOf = (spec: FieldSpec) =>
    spec.scope === "module"
      ? savedModules[spec.module!]?.[spec.name] ?? spec.base
      : spec.id in savedGlobal
      ? savedGlobal[spec.id]
      : spec.base;
  const isSaved = (spec: FieldSpec) =>
    spec.scope === "module"
      ? spec.name in (savedModules[spec.module!] ?? {})
      : spec.id in savedGlobal;

  const gFields = global ? globalFields(global) : [];
  const mFields = modules ? moduleFields(modules) : [];

  const keyValue = (key: string) => {
    if (key.includes(".")) {
      const at = key.lastIndexOf(".");
      const [module, field] = [key.slice(0, at), key.slice(at + 1)];
      const saved = savedModules[module];
      if (saved && field in saved) {
        return saved[field];
      }
      return mFields.find((spec) => spec.module === module && spec.name === field)?.base;
    }
    return key in savedGlobal ? savedGlobal[key] : global?.defaults?.[key];
  };
  const setMany = (values: Record<string, unknown>) =>
    save(async () => {
      const globals = { ...savedGlobal };
      let moduleNext = savedModules;
      for (const [key, value] of Object.entries(values)) {
        if (key.includes(".")) {
          const spec = mFields.find((one) => `${one.module}.${one.name}` === key);
          if (spec) {
            moduleNext = withModule(
              moduleNext,
              spec,
              sameValue(value, spec.base) ? undefined : value,
            );
          }
        } else if (sameValue(value, global?.defaults?.[key])) {
          delete globals[key];
        } else {
          globals[key] = value;
        }
      }
      if (!sameValue(moduleNext, savedModules)) {
        await saveModules(moduleNext);
      }
      if (!sameValue(globals, savedGlobal)) {
        await saveGlobal(globals);
      }
    });
  const shown = (setting: Recommended) =>
    !setting.when ||
    Object.entries(setting.when).every(([key, value]) => sameValue(keyValue(key), value));

  const row = (spec: FieldSpec, id: string, control?: ReactNode) => (
    <SettingRow
      key={id}
      data-field={id}
      label={spec.label ?? spec.name.replaceAll("_", " ")}
      description={spec.description}
      docs={spec.docs}
      saved={isSaved(spec)}
      resetTo={spec.base}
      onReset={() => resetField(spec)}
    >
      {control ??
        <SettingInput field={spec} value={valueOf(spec)} onChange={(v) => setField(spec, v)} />}
    </SettingRow>
  );

  const recommendedRow = (setting: Recommended) => {
    const choices = setting.choices ?? [];
    const choose = (current: number, pick: (index: number) => void) => (
      <SettingChoice
        labels={choices.map((choice) => choice.label)}
        value={current}
        onChange={pick}
        label={setting.label}
      />
    );
    if (setting.kind === "pick") {
      const current = choices.findIndex((choice) =>
        Object.entries(choice.set ?? {}).every(([key, value]) => sameValue(keyValue(key), value))
      );
      return (
        <SettingRow key={setting.id} data-field={setting.id} label={setting.label} wide>
          {choose(current, (index) => setMany(choices[index].set ?? {}))}
        </SettingRow>
      );
    }
    const fields = [...mFields, ...gFields];
    const spec = fields.find((one) => one.id === setting.key && one.scope === setting.scope) ??
      fields.find((one) =>
        setting.scope === "module" && `${one.module}.${one.name}` === setting.key
      );
    if (!spec) {
      return null;
    }
    const labelled: FieldSpec = {
      ...spec,
      label: setting.label,
      placeholder: setting.placeholder ?? spec.placeholder,
      docs: setting.docs,
    };
    const current = choices.findIndex((choice) => sameValue(choice.value, valueOf(spec)));
    return row(
      labelled,
      setting.id,
      setting.choices
        ? choose(current, (index) => setField(spec, choices[index].value))
        : undefined,
    );
  };

  const list = (fields: FieldSpec[], heading: (group: string) => string) => {
    const needle = query.trim().toLowerCase().replaceAll(" ", "_");
    const matching = fields.filter((spec) =>
      !needle || spec.id.toLowerCase().includes(needle) ||
      (spec.description ?? "").toLowerCase().includes(query.trim().toLowerCase())
    );
    if (matching.length === 0) {
      return (
        <div className="note">{query ? `Nothing matches "${query}".` : "Nothing to set."}</div>
      );
    }
    const groups: FieldSpec[][] = [];
    for (const spec of matching) {
      if (groups.at(-1)?.[0].group === spec.group) {
        groups.at(-1)!.push(spec);
      } else {
        groups.push([spec]);
      }
    }
    return groups.map((group) => (
      <SettingGroup
        key={group[0].group}
        data-group={group[0].group}
        title={group[0].groupLabel ?? heading(group[0].group)}
        count={group.filter(isSaved).length}
        accent={group[0].scope === "module"}
      >
        {group.map((spec) => row(spec, spec.id))}
      </SettingGroup>
    ));
  };

  const recommendedRows = global && modules
    ? recommended.filter(shown).map(recommendedRow).filter(Boolean)
    : [];
  const count = Object.keys(savedGlobal).length +
    Object.values(savedModules).reduce((sum, fields) => sum + Object.keys(fields).length, 0);
  return (
    <Modal
      open
      onClose={onClose}
      size="lg"
      aria-label={`Configure ${name}`}
      title={
        <>
          Config <span className="sub">{count ? `${count} set` : "dimOS defaults"}</span>
        </>
      }
      footer={<Button onClick={onClose}>Done</Button>}
      data-bp-config
    >
      <TextInput
        type="search"
        icon="search"
        placeholder="Search config…"
        aria-label="search config"
        value={query}
        onChange={setQuery}
      />
      <p className="note">Saved for every launch of this blueprint. Relaunch to apply.</p>
      {problem && <Alert tone="warn">{problem}</Alert>}
      <div className="scroll">
        {recommendedRows.length > 0 && (
          <SettingSection
            title="Recommended settings"
            hint="Decide these first: the rest can stay at dimOS's defaults."
            highlight
          >
            {recommendedRows}
          </SettingSection>
        )}
        <SettingSection title="Module config" note={name}>
          {modules
            ? list(mFields, (group) => group)
            : <div className="note">reading {name}'s modules…</div>}
        </SettingSection>
        <SettingSection title="Global config" note="every blueprint">
          {global
            ? list(gFields, (group) => group === "general" ? "general" : `${group}_*`)
            : <div className="note">reading dimOS's global config…</div>}
        </SettingSection>
      </div>
    </Modal>
  );
}
