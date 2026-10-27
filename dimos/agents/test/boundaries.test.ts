import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { isBuiltin } from "node:module";
import { dirname, join, posix, relative, sep } from "node:path";
import { test } from "node:test";
import ts from "typescript";

const SDK_BROWSER = /^sdk\/(?!node\.ts$|relay\.ts$|streams\/)/;
const BROWSER_FILES = [/^shared\/(protocol|view-models)\//, SDK_BROWSER];

/** Package files a terminal may reach through value imports. */
const TERMINAL_RUNTIME = [/^frontends\//, /^sdk\//, /^shared\/(protocol|view-models|render)\//];

const TERMINAL_PACKAGES = [
  /^@earendil-works\/chord$/,
  /^@earendil-works\/pi-tui$/,
  /^@earendil-works\/pi-coding-agent(\/|$)/,
  /^node:/,
];

const BROWSER_PACKAGES = [/^typebox$/, /^@earendil-works\/chord$/, /^@earendil-works\/pi-ai\/providers\/model-schema$/];

const UI_INTERNALS = [
  /^shared\/protocol\/schema\.ts$/,
  /^shared\/protocol\/actions\/(methods|standing)\.ts$/,
  /^sdk\/lib\//,
];

const LOCAL_LAYERS: readonly {
  readonly source: RegExp;
  readonly targets: readonly RegExp[];
  readonly typeTargets?: readonly RegExp[];
}[] = [
  { source: /^shared\/protocol\//, targets: [/^shared\/protocol\//] },
  { source: SDK_BROWSER, targets: [SDK_BROWSER, /^shared\/protocol\//] },
  {
    source: /^sdk\/(node\.ts$|relay\.ts$|streams\/)/,
    targets: [/^sdk\//, /^shared\/protocol\//],
  },
  {
    source: /^shared\/view-models\//,
    targets: [/^shared\/(view-models|protocol)\//],
    typeTargets: [/^sdk\/(main|state)\.ts$/],
  },
  { source: /^shared\/render\//, targets: [/^shared\/(render|protocol)\//] },
  {
    source: /^harness\//,
    targets: [/^harness\//, /^sdk\/(relay\.ts$|streams\/)/, /^shared\/(protocol|render)\//],
  },
  {
    source: /^gateway\//,
    targets: [/^gateway\//, /^harness\//, /^shared\/protocol\//, /^shared\/socket\.ts$/],
  },
  {
    source: /^cli\//,
    targets: [/^cli\//, /^gateway\/(config\/|main\.ts$)/, /^sdk\//, /^frontends\//, /^shared\//],
  },
  {
    source: /^frontends\//,
    targets: [/^frontends\//, /^sdk\//, /^shared\/(protocol|view-models|render)\//],
  },
];

/** The one file that calls pi-durable's `submit`. */
const SUBMITTER = "harness/inputs/gate.ts";

/** The one file that imports pi-mcp. */
const MCP_CLIENT = "harness/dimos/connection.ts";

/**
 * Where pi-coding-agent may be imported: renderers, the login flows, the one summarizer over pi-durable messages, and
 * the skill catalog's frontmatter parser.
 */
const CODING_AGENT_IMPORTERS = /^(frontends|gateway\/auth)\/|^harness\/(history\/summaries|skills\/catalog)\.ts$/;

/**
 * Every folder with a base.ts. `consumer` is the one list of the family and must import every member as a value;
 * null when each member has its own callers. `notMembers` are files of the folder that the contract does not cover.
 */
const FAMILIES: {
  readonly [directory: string]: {
    readonly consumer: string | null;
    readonly notMembers: readonly string[];
  };
} = {
  "cli/commands": { consumer: "cli/main.ts", notMembers: [] },
  "shared/protocol/actions": {
    consumer: "shared/protocol/actions/catalog.ts",
    notMembers: [
      "shared/protocol/actions/methods.ts",
      "shared/protocol/actions/request.ts",
      "shared/protocol/actions/standing.ts",
    ],
  },
  "shared/render": {
    consumer: "shared/render/render.ts",
    notMembers: [
      "shared/render/types.ts",
      "shared/render/raster.ts",
      "shared/render/frames.ts",
      "shared/render/files.ts",
    ],
  },
  "harness/dimos/wire": { consumer: null, notMembers: ["harness/dimos/wire/decoder.ts"] },
  "harness/tools": {
    consumer: "harness/main.ts",
    notMembers: ["harness/tools/details.ts", "harness/tools/extension.ts", "harness/tools/runtime-catalog.ts"],
  },
  "gateway/server/objects": {
    consumer: "gateway/server/host.ts",
    notMembers: [],
  },
  "frontends/tui/cards": {
    consumer: "frontends/tui/cards/choose.ts",
    notMembers: [],
  },
  "frontends/tui/views": {
    consumer: "frontends/tui/views/gallery.ts",
    notMembers: [],
  },
  "frontends/tui/menu": {
    consumer: "frontends/tui/menu/view.ts",
    notMembers: [],
  },
  "sdk/streams": { consumer: null, notMembers: [] },
};

type Import = {
  readonly spec: string;
  /** The imported package file, relative to the package root; null for a package import. */
  readonly target: string | null;
  readonly typeOnly: boolean;
};

/** The nearest directory at or above `from` that holds tsconfig.json: `dimos/agents`. */
function packageRoot(from: string): string {
  if (existsSync(join(from, "tsconfig.json"))) return from;
  if (dirname(from) === from) throw new Error("No tsconfig.json above the test");
  return packageRoot(dirname(from));
}

function packageProgram(root: string): ts.Program {
  const read = ts.readConfigFile(join(root, "tsconfig.json"), ts.sys.readFile);
  const config = ts.parseJsonConfigFileContent(read.config, ts.sys, root);
  return ts.createProgram(config.fileNames, config.options);
}

/** The package's own files, tests excepted, by path relative to the root. */
function packageFiles(root: string, program: ts.Program): ReadonlyMap<string, ts.SourceFile> {
  const files = new Map<string, ts.SourceFile>();
  for (const file of program.getSourceFiles()) {
    const path = relative(root, file.fileName).split(sep).join("/");
    if (path.startsWith("../") || path.includes("node_modules/") || path.endsWith(".test.ts")) continue;
    files.set(path, file);
  }
  return files;
}

function importsOf(path: string, file: ts.SourceFile): Import[] {
  const found = new Map<string, Import>();
  const add = (spec: string, typeOnly: boolean): void => {
    const target = spec.startsWith(".") ? posix.normalize(posix.join(posix.dirname(path), spec)) : null;
    found.set(`${typeOnly}:${spec}`, { spec, target, typeOnly });
  };
  const visit = (node: ts.Node): void => {
    if (ts.isImportDeclaration(node) && ts.isStringLiteral(node.moduleSpecifier)) {
      const clause = node.importClause;
      const named =
        clause?.namedBindings !== undefined && ts.isNamedImports(clause.namedBindings)
          ? clause.namedBindings.elements
          : null;
      const typeOnly =
        clause !== undefined &&
        (clause.phaseModifier === ts.SyntaxKind.TypeKeyword ||
          (clause.name === undefined &&
            named !== null &&
            named.length > 0 &&
            named.every((element) => element.isTypeOnly)));
      add(node.moduleSpecifier.text, typeOnly);
    } else if (
      ts.isCallExpression(node) &&
      node.expression.kind === ts.SyntaxKind.ImportKeyword &&
      node.arguments[0] !== undefined &&
      ts.isStringLiteralLike(node.arguments[0])
    ) {
      add(node.arguments[0].text, false);
    } else if (
      ts.isImportTypeNode(node) &&
      ts.isLiteralTypeNode(node.argument) &&
      ts.isStringLiteralLike(node.argument.literal)
    ) {
      add(node.argument.literal.text, true);
    }
    ts.forEachChild(node, visit);
  };
  visit(file);
  return [...found.values()];
}

/** `export … from`, and `export { name }` of an imported name. */
function reexports(file: ts.SourceFile): boolean {
  const imported = new Set<string>();
  for (const statement of file.statements) {
    const clause = ts.isImportDeclaration(statement) ? statement.importClause : undefined;
    if (clause?.name) imported.add(clause.name.text);
    if (clause?.namedBindings) {
      if (ts.isNamedImports(clause.namedBindings))
        clause.namedBindings.elements.forEach((element) => imported.add(element.name.text));
      else imported.add(clause.namedBindings.name.text);
    }
  }
  return file.statements.some(
    (statement) =>
      ts.isExportDeclaration(statement) &&
      (statement.moduleSpecifier !== undefined ||
        (statement.exportClause !== undefined &&
          ts.isNamedExports(statement.exportClause) &&
          statement.exportClause.elements.some((element) =>
            imported.has((element.propertyName ?? element.name).getText(file)),
          ))),
  );
}

function isDurableDeclaration(declaration: ts.Declaration): boolean {
  return declaration.getSourceFile().fileName.includes("/@earendil-works/pi-durable/");
}

/** Every use of a `submit` member that pi-durable declares (Conversation, ConversationHandle and the rest), as `path:line`. */
function durableSubmits(program: ts.Program, files: ReadonlyMap<string, ts.SourceFile>): string[] {
  const checker = program.getTypeChecker();
  const found: string[] = [];
  for (const [path, file] of files) {
    const visit = (node: ts.Node): void => {
      const declarations =
        ts.isPropertyAccessExpression(node) && node.name.text === "submit"
          ? checker.getSymbolAtLocation(node.name)?.declarations
          : ts.isElementAccessExpression(node) &&
              ts.isStringLiteral(node.argumentExpression) &&
              node.argumentExpression.text === "submit"
            ? checker.getNonNullableType(checker.getTypeAtLocation(node.expression)).getProperty("submit")?.declarations
            : undefined;
      if (declarations?.some(isDurableDeclaration))
        found.push(`${path}:${file.getLineAndCharacterOfPosition(node.getStart(file)).line + 1}`);
      ts.forEachChild(node, visit);
    };
    visit(file);
  }
  return found;
}

/** Each import cycle once, as `a -> b -> a`, over every relative import, type-only ones included. */
function cycles(imports: ReadonlyMap<string, readonly Import[]>): string[] {
  const found = new Set<string>();
  const done = new Set<string>();
  const walk = (path: string, stack: readonly string[]): void => {
    if (stack.includes(path)) {
      const loop = stack.slice(stack.indexOf(path));
      const start = loop.indexOf([...loop].sort()[0] ?? path);
      const canonical = [...loop.slice(start), ...loop.slice(0, start)];
      found.add([...canonical, canonical[0]].join(" -> "));
      return;
    }
    if (done.has(path)) return;
    for (const item of imports.get(path) ?? [])
      if (item.target !== null && imports.has(item.target)) walk(item.target, [...stack, path]);
    done.add(path);
  };
  for (const path of imports.keys()) walk(path, []);
  return [...found].sort();
}

/** Package files a terminal runs: everything reachable from frontends/ through value imports. */
function terminalRuntime(imports: ReadonlyMap<string, readonly Import[]>): Map<string, string> {
  const reachedFrom = new Map<string, string>();
  const queue = [...imports.keys()].filter((path) => path.startsWith("frontends/"));
  for (const path of queue) reachedFrom.set(path, path);
  while (queue.length > 0) {
    const path = queue.shift() ?? "";
    for (const item of imports.get(path) ?? []) {
      if (item.typeOnly || item.target === null || reachedFrom.has(item.target)) continue;
      reachedFrom.set(item.target, path);
      queue.push(item.target);
    }
  }
  return reachedFrom;
}

/** Every broken rule, one line each, naming the file, the import and the rule. */
function boundaryViolations(root: string): string[] {
  const program = packageProgram(root);
  const files = packageFiles(root, program);
  const imports = new Map([...files].map(([path, file]) => [path, importsOf(path, file)] as const));
  const problems: string[] = [];

  if (!program.getSourceFiles().some((file) => file.fileName.includes("/@earendil-works/pi-durable/"))) {
    problems.push("pi-durable's declarations are not in the program, so the submit() check would pass vacuously");
  }

  for (const [path, file] of files) {
    if (posix.basename(path) === "index.ts") problems.push(`${path}: an index file`);
    if (reexports(file)) problems.push(`${path}: re-exports; import from the defining file`);
    for (const item of imports.get(path) ?? []) {
      const { spec, target } = item;
      if (spec.startsWith("@earendil-works/pi-mcp") && path !== MCP_CLIENT)
        problems.push(`${path}: imports ${spec}; only ${MCP_CLIENT} may`);
      if (spec.startsWith("@earendil-works/pi-coding-agent") && !CODING_AGENT_IMPORTERS.test(path))
        problems.push(
          `${path}: imports ${spec}; only frontends/, gateway/auth/, harness/history/summaries.ts and harness/skills/catalog.ts may`,
        );
      if (BROWSER_FILES.some((browser) => browser.test(path))) {
        if (spec.startsWith("node:") || isBuiltin(spec))
          problems.push(`${path}: imports ${spec}; this code runs in a browser too`);
        if (target === null && !item.typeOnly && !BROWSER_PACKAGES.some((allowed) => allowed.test(spec))) {
          problems.push(
            `${path}: imports values from ${spec}; browser code uses only typebox, chord and the model schema`,
          );
        }
      }
      if (path.startsWith("frontends/") && target !== null && UI_INTERNALS.some((internal) => internal.test(target)))
        problems.push(`${path}: imports ${target}; a renderer uses the SDK and view-models, not their internals`);
      if (
        spec.startsWith("@earendil-works/pi-durable") &&
        !item.typeOnly &&
        /^(frontends|shared\/render)\//.test(path)
      ) {
        problems.push(`${path}: imports values from ${spec}; frontends/ and shared/render/ may import its types only`);
      }
      if (
        path.startsWith("frontends/") &&
        target === null &&
        !item.typeOnly &&
        !TERMINAL_PACKAGES.some((allowed) => allowed.test(spec))
      ) {
        problems.push(
          `${path}: imports values from ${spec}; a terminal runs only chord, pi-tui, pi-coding-agent and node`,
        );
      }
      if (target === null) continue;
      if (!files.has(target)) problems.push(`${path}: imports ${spec}, which does not exist`);
      const layer = LOCAL_LAYERS.find((rule) => rule.source.test(path));
      if (
        layer !== undefined &&
        !layer.targets.some((allowed) => allowed.test(target)) &&
        !(item.typeOnly && layer.typeTargets?.some((allowed) => allowed.test(target)))
      ) {
        problems.push(`${path}: imports ${target}; outside its allowed layer dependencies`);
      }
      if (target === "shared/socket.ts" && !/^(gateway|cli)\//.test(path))
        problems.push(`${path}: imports ${target}; only gateway/ and cli/ use the socket helper`);
      const parts = target.split("/");
      const lib = parts.indexOf("lib");
      if (lib >= 0 && !path.startsWith(`${parts.slice(0, lib).join("/")}/`))
        problems.push(`${path}: imports ${target}; a lib/ serves only its own folder`);
    }
  }

  for (const [path, from] of terminalRuntime(imports)) {
    if (!TERMINAL_RUNTIME.some((allowed) => allowed.test(path)))
      problems.push(`${from}: makes a terminal run ${path}; import its types only`);
  }

  for (const use of durableSubmits(program, files)) {
    if (!use.startsWith(`${SUBMITTER}:`))
      problems.push(`${use}: uses pi-durable's submit(); every input goes through ${SUBMITTER}`);
  }

  for (const cycle of cycles(imports)) problems.push(`import cycle: ${cycle}`);

  const familyDirectories = new Set(
    [...files.keys()].filter((path) => posix.basename(path) === "base.ts").map((path) => posix.dirname(path)),
  );
  for (const directory of Object.keys(FAMILIES)) {
    if (!familyDirectories.has(directory)) problems.push(`${directory}: listed in FAMILIES but has no base.ts`);
  }
  for (const directory of familyDirectories) {
    const family = FAMILIES[directory];
    if (family === undefined) {
      problems.push(`${directory}/base.ts: a family with no FAMILIES entry`);
      continue;
    }
    for (const path of [family.consumer, ...family.notMembers])
      if (path !== null && !files.has(path)) problems.push(`${path}: listed in FAMILIES but missing`);
    const listed = new Set(
      (imports.get(family.consumer ?? "") ?? []).filter((item) => !item.typeOnly).map((item) => item.target),
    );
    const members = [...files.keys()].filter(
      (path) =>
        posix.dirname(path) === directory &&
        path !== `${directory}/base.ts` &&
        path !== family.consumer &&
        !family.notMembers.includes(path),
    );
    for (const member of members) {
      if (!(imports.get(member) ?? []).some((item) => item.target === `${directory}/base.ts`))
        problems.push(`${member}: does not implement ${directory}/base.ts`);
      if (family.consumer !== null && !listed.has(member)) problems.push(`${member}: missing from ${family.consumer}`);
    }
  }
  return problems;
}

test("imports respect the package boundaries, the family contracts and the one submitter", () => {
  assert.deepEqual(boundaryViolations(packageRoot(import.meta.dirname)), []);
});

/** Chord refuses a state member that shares its name with a method at `remote()` time; this says which one, before a gateway starts. */
test("no served contract names a state member and a method alike", async () => {
  const contracts = [
    (await import("../shared/protocol/conversation/contract.ts")).ConversationContract,
    (await import("../shared/protocol/gateway/contract.ts")).GatewayContract,
    (await import("../shared/protocol/lifecycle/contract.ts")).LifecycleContract,
    (await import("../shared/protocol/inventory/contract.ts")).InventoryContract,
  ];
  const clashes: string[] = [];
  for (const contract of contracts) {
    for (const [variant, def] of Object.entries(contract.def.variants)) {
      const methods = Object.keys(def.methods ?? {});
      for (const member of Object.keys(def.state ?? {}))
        if (methods.includes(member)) clashes.push(`${contract.name}/${variant}: ${member}`);
    }
  }
  assert.deepEqual(clashes, []);
});
