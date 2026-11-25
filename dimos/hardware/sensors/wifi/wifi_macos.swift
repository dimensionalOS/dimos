// Copyright 2026 Dimensional Inc. Licensed under Apache-2.0.
// Read current interface state without initiating a network scan.
import Foundation
import CoreWLAN

let name = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "en0"
if let radio = CWWiFiClient.shared().interface(withName: name) {
    let signal = radio.rssiValue()
    let channel = radio.wlanChannel()
    let data: [String: Any] = [
        "interface": name, "source": "macOS CoreWLAN", "counters": [:],
        "associated": channel != nil, "bssid": radio.bssid() as Any? ?? NSNull(),
        "signal_dbm": signal != 0 ? signal as Any : NSNull(),
        "noise_dbm": signal != 0 ? radio.noiseMeasurement() as Any : NSNull(),
        "tx_rate_mbps": channel != nil ? radio.transmitRate() as Any : NSNull(),
        "channel": channel != nil ? channel!.channelNumber as Any : NSNull()
    ]
    let output = try JSONSerialization.data(withJSONObject: data, options: [.sortedKeys])
    print(String(data: output, encoding: .utf8)!)
} else {
    print("{\"status\":\"unavailable\",\"reason\":\"wireless interface unavailable\"}")
}
