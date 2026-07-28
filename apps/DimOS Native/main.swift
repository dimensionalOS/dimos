import AppKit
import Foundation

private let studioBlueprint = "dimos-go2-studio.go2"

final class AppDelegate: NSObject, NSApplicationDelegate {
    private var window: NSWindow!
    private let robotIPField = NSTextField(string: "192.168.12.1")
    private let statusView = NSTextView()
    private var projectRoot: URL?
    private var runtimeProcess: Process?
    private var isTransitioning = false

    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApp.setActivationPolicy(.regular)
        projectRoot = locateProjectRoot()
        buildWindow()
        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)

        if let root = projectRoot {
            appendStatus("项目：\(root.path)")
            appendStatus("原生渲染：dimos-viewer（Rerun / Metal）")
            appendStatus("官方 MCP：http://127.0.0.1:9990/mcp")
            refreshStatus(nil)
        } else {
            appendStatus("错误：找不到包含 .venv/bin/dimos 的项目目录。")
        }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        true
    }

    func applicationWillTerminate(_ notification: Notification) {
        if runtimeProcess?.isRunning == true {
            runtimeProcess?.terminate()
        }
    }

    private func buildWindow() {
        window = NSWindow(
            contentRect: NSRect(x: 0, y: 0, width: 760, height: 560),
            styleMask: [.titled, .closable, .miniaturizable, .resizable],
            backing: .buffered,
            defer: false
        )
        window.title = "DimOS Native"
        window.center()

        let title = NSTextField(labelWithString: "DimOS 原生控制台")
        title.font = NSFont.systemFont(ofSize: 24, weight: .bold)

        let subtitle = NSTextField(
            wrappingLabelWithString:
                "地图与相机默认使用原生 dimos-viewer，不启动浏览器 WebGL。"
        )
        subtitle.textColor = .secondaryLabelColor

        let ipLabel = NSTextField(labelWithString: "机器狗 IP")
        ipLabel.font = NSFont.systemFont(ofSize: 13, weight: .semibold)
        robotIPField.placeholderString = "192.168.12.1"

        let inputGrid = NSGridView(views: [
            [ipLabel, robotIPField],
        ])
        inputGrid.rowSpacing = 10
        inputGrid.columnSpacing = 12
        inputGrid.column(at: 0).xPlacement = .trailing
        inputGrid.column(at: 1).width = 430

        let replayButton = button(
            "回放演示（不连接狗）",
            action: #selector(startReplay(_:))
        )
        let readOnlyButton = button(
            "只读连接（不能遥控）",
            action: #selector(startReadOnly(_:))
        )
        let hardwareButton = button(
            "连接并启用遥控",
            action: #selector(startHardware(_:))
        )
        let stopButton = button("停止 DimOS", action: #selector(stopDimOS(_:)))
        stopButton.contentTintColor = .systemRed

        let nativeRow = NSStackView(
            views: [replayButton, readOnlyButton, hardwareButton, stopButton]
        )
        nativeRow.orientation = .horizontal
        nativeRow.spacing = 10
        nativeRow.distribution = .fillEqually

        let viewerButton = button(
            "重新打开原生 Viewer",
            action: #selector(openNativeViewer(_:))
        )
        let commandCenterButton = button(
            "打开原系统操作界面",
            action: #selector(openCommandCenter(_:))
        )
        let studioButton = button(
            "打开 Studio（有头浏览器）",
            action: #selector(openHeadedStudio(_:))
        )
        let mcpButton = button("检查官方 MCP", action: #selector(checkMCP(_:)))
        let trajectoryButton = button(
            "查看实际轨迹",
            action: #selector(checkRobotSummary(_:))
        )
        let statusButton = button("刷新状态", action: #selector(refreshStatus(_:)))

        let toolsRow = NSStackView(
            views: [
                viewerButton,
                commandCenterButton,
                studioButton,
                mcpButton,
                trajectoryButton,
                statusButton,
            ]
        )
        toolsRow.orientation = .horizontal
        toolsRow.spacing = 8
        toolsRow.distribution = .fillEqually

        statusView.isEditable = false
        statusView.font = NSFont.monospacedSystemFont(ofSize: 12, weight: .regular)
        statusView.textContainerInset = NSSize(width: 10, height: 10)
        statusView.backgroundColor = NSColor.textBackgroundColor

        let scrollView = NSScrollView()
        scrollView.hasVerticalScroller = true
        scrollView.borderType = .bezelBorder
        scrollView.documentView = statusView
        scrollView.heightAnchor.constraint(greaterThanOrEqualToConstant: 230).isActive = true

        let stack = NSStackView(
            views: [title, subtitle, inputGrid, nativeRow, toolsRow, scrollView]
        )
        stack.orientation = .vertical
        stack.alignment = .leading
        stack.spacing = 14
        stack.translatesAutoresizingMaskIntoConstraints = false

        guard let content = window.contentView else { return }
        content.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.leadingAnchor.constraint(equalTo: content.leadingAnchor, constant: 22),
            stack.trailingAnchor.constraint(equalTo: content.trailingAnchor, constant: -22),
            stack.topAnchor.constraint(equalTo: content.topAnchor, constant: 22),
            stack.bottomAnchor.constraint(equalTo: content.bottomAnchor, constant: -22),
            inputGrid.widthAnchor.constraint(equalTo: stack.widthAnchor),
            nativeRow.widthAnchor.constraint(equalTo: stack.widthAnchor),
            toolsRow.widthAnchor.constraint(equalTo: stack.widthAnchor),
            scrollView.widthAnchor.constraint(equalTo: stack.widthAnchor),
        ])
    }

    private func button(_ title: String, action: Selector) -> NSButton {
        let result = NSButton(title: title, target: self, action: action)
        result.bezelStyle = .rounded
        result.controlSize = .large
        return result
    }

    private func locateProjectRoot() -> URL? {
        if let configured = ProcessInfo.processInfo.environment["DIMOS_PROJECT_ROOT"] {
            let candidate = URL(fileURLWithPath: configured)
            if FileManager.default.isExecutableFile(
                atPath: candidate.appendingPathComponent(".venv/bin/dimos").path
            ) {
                return candidate
            }
        }

        var candidate = Bundle.main.bundleURL
        for _ in 0..<8 {
            candidate.deleteLastPathComponent()
            let executable = candidate.appendingPathComponent(".venv/bin/dimos")
            if FileManager.default.isExecutableFile(atPath: executable.path) {
                return candidate
            }
        }
        return nil
    }

    private func appendStatus(_ text: String) {
        DispatchQueue.main.async {
            let timestamp = DateFormatter.localizedString(
                from: Date(),
                dateStyle: .none,
                timeStyle: .medium
            )
            self.statusView.string += "[\(timestamp)] \(text)\n"
            self.statusView.scrollToEndOfDocument(nil)
        }
    }

    private func dimosEnvironment(root: URL) -> [String: String] {
        var environment = ProcessInfo.processInfo.environment
        let bin = root.appendingPathComponent(".venv/bin").path
        environment["PATH"] = "\(bin):\(environment["PATH"] ?? "/usr/bin:/bin")"
        environment["PYTHONUNBUFFERED"] = "1"
        let bypasses = ["localhost", "127.0.0.1", robotIPField.stringValue]
        let existingBypasses = environment["NO_PROXY"] ?? environment["no_proxy"] ?? ""
        environment["NO_PROXY"] = ([existingBypasses] + bypasses)
            .filter { !$0.isEmpty }
            .joined(separator: ",")
        environment["no_proxy"] = environment["NO_PROXY"]
        return environment
    }

    private func runProcess(
        executable: URL,
        arguments: [String],
        completion: ((Int32, String) -> Void)? = nil
    ) {
        guard let root = projectRoot else {
            appendStatus("无法执行：项目目录不可用。")
            return
        }

        let process = Process()
        let pipe = Pipe()
        process.executableURL = executable
        process.arguments = arguments
        process.currentDirectoryURL = root
        process.environment = dimosEnvironment(root: root)
        process.standardOutput = pipe
        process.standardError = pipe
        process.terminationHandler = { process in
            let data = pipe.fileHandleForReading.readDataToEndOfFile()
            let output = String(data: data, encoding: .utf8) ?? ""
            DispatchQueue.main.async {
                completion?(process.terminationStatus, output.trimmingCharacters(in: .whitespacesAndNewlines))
            }
        }

        do {
            try process.run()
        } catch {
            appendStatus("启动失败：\(error.localizedDescription)")
        }
    }

    private func runDimos(
        _ arguments: [String],
        completion: ((Int32, String) -> Void)? = nil
    ) {
        guard let root = projectRoot else {
            appendStatus("DimOS 路径不可用。")
            return
        }
        runProcess(
            executable: root.appendingPathComponent(".venv/bin/dimos"),
            arguments: arguments,
            completion: completion
        )
    }

    private func launchRuntime(_ arguments: [String]) {
        guard let root = projectRoot else {
            appendStatus("DimOS 路径不可用。")
            return
        }

        let logURL = root.appendingPathComponent("logs/dimos-native-runtime.log")
        FileManager.default.createFile(atPath: logURL.path, contents: nil)
        guard let logHandle = try? FileHandle(forWritingTo: logURL) else {
            appendStatus("无法打开运行日志：\(logURL.path)")
            return
        }
        _ = try? logHandle.seekToEnd()

        let process = Process()
        process.executableURL = root.appendingPathComponent(".venv/bin/dimos")
        process.arguments = arguments
        process.currentDirectoryURL = root
        process.environment = dimosEnvironment(root: root)
        process.standardOutput = logHandle
        process.standardError = logHandle
        process.terminationHandler = { [weak self] process in
            try? logHandle.close()
            DispatchQueue.main.async {
                if self?.runtimeProcess === process {
                    self?.runtimeProcess = nil
                }
                self?.appendStatus("DimOS 进程已退出（\(process.terminationStatus)）。")
            }
        }

        do {
            try process.run()
            runtimeProcess = process
            appendStatus("DimOS 已启动，PID \(process.processIdentifier)。")
            DispatchQueue.main.asyncAfter(deadline: .now() + 12) {
                self.refreshStatus(nil)
            }
        } catch {
            try? logHandle.close()
            appendStatus("启动失败：\(error.localizedDescription)")
        }
    }

    private func prepareRuntime(completion: @escaping () -> Void) {
        guard let root = projectRoot else {
            appendStatus("项目目录不可用。")
            return
        }
        guard !isTransitioning else {
            appendStatus("正在切换 DimOS 运行模式，请稍候。")
            return
        }

        isTransitioning = true
        let helper = root.appendingPathComponent("scripts/stop_dimos_native_conflicts.sh")
        appendStatus("正在停止旧 DimOS/MCP，并释放控制端口…")
        runProcess(
            executable: URL(fileURLWithPath: "/bin/zsh"),
            arguments: [helper.path]
        ) { code, output in
            self.runtimeProcess = nil
            self.isTransitioning = false
            guard code == 0 else {
                self.appendStatus(
                    "无法切换运行模式（\(code)）：\(output.isEmpty ? "控制端口仍被占用" : output)"
                )
                return
            }
            self.appendStatus(output.isEmpty ? "旧运行实例已停止。" : output)
            completion()
        }
    }

    private func startRuntime(readOnly: Bool) {
        let robotIP = robotIPField.stringValue.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !robotIP.isEmpty else {
            appendStatus("请输入机器狗 IP。")
            return
        }
        var arguments = [
            "--robot-ip", robotIP,
            "--viewer", "rerun",
            "--rerun-open", "native",
            "--no-rerun-web",
            "--obstacle-avoidance",
            "run", studioBlueprint,
        ]
        if readOnly {
            arguments += [
                "--option", "go2connection.movement_enabled=false",
                "--option", "go2connection.auto_stand=false",
            ]
        } else {
            arguments += [
                "--option", "go2connection.movement_enabled=true",
                "--option", "go2connection.auto_stand=false",
            ]
        }

        appendStatus(
            readOnly
                ? "正在启动只读模式：地图/相机可用，键盘移动会被拦截。"
                : "正在启动可遥控模式：Viewer 打开后请先点击画面，再用键盘控制。"
        )
        prepareRuntime {
            self.launchRuntime(arguments)
        }
    }

    @objc private func startReplay(_ sender: Any?) {
        appendStatus("正在启动官方 Go2 回放、原生 Viewer 与 MCP…")
        prepareRuntime {
            self.launchRuntime([
                "--replay",
                "--viewer", "rerun",
                "--rerun-open", "native",
                "--no-rerun-web",
                "run", studioBlueprint,
            ])
        }
    }

    @objc private func startReadOnly(_ sender: Any?) {
        startRuntime(readOnly: true)
    }

    @objc private func startHardware(_ sender: Any?) {
        startRuntime(readOnly: false)
    }

    @objc private func stopDimOS(_ sender: Any?) {
        prepareRuntime {
            self.appendStatus("DimOS、旧 MCP 和原生 Viewer 已停止。")
            self.refreshStatus(nil)
        }
    }

    @objc private func refreshStatus(_ sender: Any?) {
        runDimos(["status"]) { code, output in
            self.appendStatus("DimOS 状态（\(code)）：\(output.isEmpty ? "无运行实例" : output)")
        }
    }

    @objc private func checkMCP(_ sender: Any?) {
        runDimos(["mcp", "list-tools"]) { code, output in
            self.appendStatus("官方 MCP（\(code)）：\(output.isEmpty ? "未启动" : output)")
        }
    }

    @objc private func checkRobotSummary(_ sender: Any?) {
        runDimos(["mcp", "call", "get_robot_summary"]) { code, output in
            self.appendStatus(
                "真实里程计 / actual path（\(code)）：\(output.isEmpty ? "暂无数据" : output)"
            )
        }
    }

    @objc private func openNativeViewer(_ sender: Any?) {
        guard let root = projectRoot else {
            appendStatus("项目目录不可用。")
            return
        }
        appendStatus("正在连接原生 dimos-viewer（Metal 渲染）…")
        runProcess(
            executable: root.appendingPathComponent(".venv/bin/dimos-viewer"),
            arguments: [
                "--connect", "rerun+http://127.0.0.1:9877/proxy",
                "--ws-url", "ws://127.0.0.1:3030/ws",
            ]
        )
    }

    @objc private func openCommandCenter(_ sender: Any?) {
        guard let url = URL(string: "http://127.0.0.1:7779/command-center") else { return }
        NSWorkspace.shared.open(url)
        appendStatus("已在有头浏览器打开原系统操作界面。")
    }

    @objc private func openHeadedStudio(_ sender: Any?) {
        guard let root = projectRoot else {
            appendStatus("项目目录不可用。")
            return
        }
        let launcher = root.appendingPathComponent("scripts/open_dimos_studio.command")
        runProcess(executable: URL(fileURLWithPath: "/bin/zsh"), arguments: [launcher.path])
        appendStatus("正在启动 Studio 并打开普通浏览器窗口。")
    }
}

let application = NSApplication.shared
let delegate = AppDelegate()
application.delegate = delegate
application.run()
