const { execSync } = require("child_process");
const fs = require("fs");
const path = require("path");

const dataDir = path.join(__dirname, "..", "..", "..", "..", "data");
const outPath = path.join(dataDir, "command_center.html");
const archivePath = path.join(dataDir, ".lfs", "command_center.html.tar.gz");

// Inline JS into HTML
const distDir = path.join(__dirname, "..", "dist-standalone");
const jsFile = fs.readdirSync(path.join(distDir, "assets")).find((f) => f.startsWith("main") && f.endsWith(".js"));
const html = fs.readFileSync(path.join(distDir, "index.html"), "utf8");
const js = fs.readFileSync(path.join(distDir, "assets", jsFile), "utf8");

const scriptTag = `<script type="module" crossorigin src="/assets/${jsFile}"></script>`;
const idx = html.indexOf(scriptTag);
const output = html.slice(0, idx) + `<script type="module">${js}</script>` + html.slice(idx + scriptTag.length);

fs.writeFileSync(outPath, output);
console.log(`Created ${outPath}`);

// Recreate the LFS archive
if (fs.existsSync(archivePath)) {
  fs.unlinkSync(archivePath);
}

execSync(`tar -czf "${archivePath}" -C "${dataDir}" command_center.html`);
console.log(`Created ${archivePath}`);
