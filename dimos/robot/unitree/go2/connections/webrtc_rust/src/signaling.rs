// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! The Go2's signaling handshake over its HTTP port, as the Unitree app does it:
//! `con_notify` (RSA public key, AES-128-GCM wrapped on newer firmware), then the SDP offer
//! AES-256-ECB under a fresh session key that rides RSA-wrapped, posted to `con_ing_*`.

use std::time::Duration;

use aes::cipher::{BlockDecryptMut, BlockEncryptMut, KeyInit};
use aes_gcm::aead::Aead;
use aes_gcm::{Aes128Gcm, Nonce};
use base64::engine::general_purpose::STANDARD as B64;
use base64::Engine;
use rand::Rng;
use rsa::pkcs8::DecodePublicKey;
use rsa::{Pkcs1v15Encrypt, RsaPublicKey};
use serde_json::{json, Value};
use tracing::debug;

pub const SIGNALING_PORT: u16 = 9991;
/// Firmware before 1.1.11 takes a plaintext offer here instead.
pub const LEGACY_SIGNALING_PORT: u16 = 8081;
/// The dog's address on its own hotspot.
const AP_MODE_IP: &str = "192.168.12.1";
/// data2=2 firmware wraps con_notify in AES-128-GCM under this fixed key; data2=3 (>= 1.1.15) uses a per-device key.
const LEGACY_GCM_KEY: [u8; 16] = [
    232, 86, 130, 189, 22, 84, 155, 0, 142, 4, 166, 104, 43, 179, 235, 227,
];

fn gcm_decrypt(data1_b64: &str, key: &[u8; 16]) -> Result<String, String> {
    let raw = B64
        .decode(data1_b64.trim())
        .map_err(|e| format!("con_notify data1 isn't base64: {e}"))?;
    if raw.len() < 28 {
        return Err("con_notify data1 is too short".into());
    }
    // the robot's layout: ciphertext, 12-byte nonce, 16-byte tag
    let (rest, tag) = raw.split_at(raw.len() - 16);
    let (ciphertext, nonce) = rest.split_at(rest.len() - 12);
    let sealed = [ciphertext, tag].concat();
    let cipher = Aes128Gcm::new_from_slice(key).map_err(|e| e.to_string())?;
    let plain = cipher
        .decrypt(Nonce::from_slice(nonce), sealed.as_slice())
        .map_err(|_| "AES-128 key rejected by the robot (GCM tag check failed)".to_string())?;
    String::from_utf8(plain).map_err(|e| e.to_string())
}

pub fn parse_aes_key(hex: &str) -> Result<[u8; 16], String> {
    let clean = hex.trim().to_lowercase();
    if clean.len() != 32 || !clean.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err("aes_128_key must be 32 hex characters (16 bytes)".into());
    }
    let bytes = (0..16)
        .map(|i| u8::from_str_radix(&clean[i * 2..i * 2 + 2], 16).map_err(|e| e.to_string()))
        .collect::<Result<Vec<u8>, String>>()?;
    Ok(bytes.try_into().expect("32 hex characters are 16 bytes"))
}

fn decrypt_data1(data1: &str, data2: i64, aes_key: &str) -> Result<String, String> {
    match data2 {
        2 => gcm_decrypt(data1, &LEGACY_GCM_KEY),
        3 if aes_key.is_empty() => Err(
            "this Go2 speaks data2=3 (firmware >= 1.1.15): set its per-device aes_128_key (GO2CONNECTION__AES_128_KEY)".into(),
        ),
        3 => gcm_decrypt(data1, &parse_aes_key(aes_key)?),
        _ => Ok(data1.to_string()),
    }
}

/// The con_ing path suffix: digits from the last 10 chars of data1 (every second char, as an index into A..J).
fn path_ending(data1: &str) -> String {
    let chars: Vec<char> = data1.chars().collect();
    let last10 = &chars[chars.len().saturating_sub(10)..];
    last10
        .chunks(2)
        .filter_map(|pair| pair.get(1).and_then(|c| "ABCDEFGHIJ".find(*c)))
        .map(|index| index.to_string())
        .collect()
}

/// AES-256-ECB/PKCS7 under a 32-char key used as UTF-8 bytes, base64 (what crypto-js does for the browser client).
fn aes_ecb_encrypt(plain: &str, key: &str) -> String {
    let cipher =
        ecb::Encryptor::<aes::Aes256>::new_from_slice(key.as_bytes()).expect("32-byte key");
    B64.encode(cipher.encrypt_padded_vec_mut::<aes::cipher::block_padding::Pkcs7>(plain.as_bytes()))
}

fn aes_ecb_decrypt(b64: &str, key: &str) -> Result<String, String> {
    let raw = B64
        .decode(b64.trim())
        .map_err(|e| format!("answer isn't base64: {e}"))?;
    let cipher =
        ecb::Decryptor::<aes::Aes256>::new_from_slice(key.as_bytes()).expect("32-byte key");
    let plain = cipher
        .decrypt_padded_vec_mut::<aes::cipher::block_padding::Pkcs7>(&raw)
        .map_err(|_| "couldn't decrypt the robot's answer".to_string())?;
    String::from_utf8(plain).map_err(|e| e.to_string())
}

fn rsa_encrypt(data: &str, public_key_b64_der: &str) -> Result<String, String> {
    let der = B64
        .decode(public_key_b64_der.trim())
        .map_err(|e| format!("robot public key isn't base64: {e}"))?;
    let key =
        RsaPublicKey::from_public_key_der(&der).map_err(|e| format!("robot public key: {e}"))?;
    let encrypted = key
        .encrypt(&mut rand::thread_rng(), Pkcs1v15Encrypt, data.as_bytes())
        .map_err(|e| e.to_string())?;
    Ok(B64.encode(encrypted))
}

/// The offer's id: "STA_localNetwork" when the dog is on a shared Wi-Fi, empty on its own hotspot.
pub fn offer_id(ip: &str) -> &'static str {
    if ip == AP_MODE_IP {
        ""
    } else {
        "STA_localNetwork"
    }
}

/// The data channel's validation answer to the robot's challenge `key`.
pub fn validation_reply(key: &str) -> String {
    B64.encode(md5::compute(format!("UnitreeGo2_{key}")).0)
}

async fn post(
    url: &str,
    content_type: &str,
    body: Option<String>,
    timeout: Duration,
) -> Result<String, String> {
    let client = reqwest::Client::builder()
        .timeout(timeout)
        .build()
        .map_err(|e| e.to_string())?;
    let mut request = client.post(url).header("content-type", content_type);
    if let Some(body) = body {
        request = request.body(body);
    }
    let response = request
        .send()
        .await
        .map_err(|e| format!("{url} unreachable: {e}"))?;
    let status = response.status();
    let text = response.text().await.map_err(|e| e.to_string())?;
    debug!(url, %status, bytes = text.len(), "signaling reply");
    if !status.is_success() {
        return Err(format!("signaling {url} failed ({status})"));
    }
    Ok(text)
}

/// Hands the robot our SDP offer (JSON `{id, sdp, type, token}`) and returns its answer's SDP.
pub async fn exchange_sdp(
    ip: &str,
    aes_key: &str,
    offer_json: &str,
    timeout: Duration,
) -> Result<String, String> {
    let notify_url = format!("http://{ip}:{SIGNALING_PORT}/con_notify");
    let notify_b64 = match post(&notify_url, "application/json", None, timeout).await {
        Ok(notify) => notify,
        // A device key means new firmware: no plaintext fallback a forged notify error could force.
        Err(err) if !aes_key.is_empty() => return Err(err),
        Err(err) => {
            return legacy_offer(ip, offer_json, timeout)
                .await
                .map_err(|legacy| format!("{err}; {legacy}"))
        }
    };
    let notify_raw = B64
        .decode(notify_b64.trim())
        .map_err(|e| format!("con_notify isn't base64: {e}"))?;
    let notify: Value =
        serde_json::from_slice(&notify_raw).map_err(|e| format!("con_notify isn't JSON: {e}"))?;
    let data1 = notify["data1"].as_str().ok_or("con_notify has no data1")?;
    let data2 = notify["data2"].as_i64().unwrap_or(0);
    debug!(data2, "con_notify");
    let data1 = decrypt_data1(data1, data2, aes_key)?;
    if data1.len() < 20 {
        return Err("con_notify data1 is too short".into());
    }
    let public_key = data1
        .get(10..data1.len() - 10)
        .ok_or("con_notify data1 isn't text")?;
    let session_key: String = (0..16)
        .map(|_| format!("{:02x}", rand::thread_rng().gen::<u8>()))
        .collect();
    let body = json!({
        "data1": aes_ecb_encrypt(offer_json, &session_key),
        "data2": rsa_encrypt(&session_key, public_key)?,
    });
    let answer = post(
        &format!(
            "http://{ip}:{SIGNALING_PORT}/con_ing_{}",
            path_ending(&data1)
        ),
        "application/x-www-form-urlencoded",
        Some(body.to_string()),
        timeout,
    )
    .await?;
    answer_sdp(&aes_ecb_decrypt(&answer, &session_key)?)
}

/// Firmware before 1.1.11: the offer goes in plaintext and the answer comes back the same way.
async fn legacy_offer(ip: &str, offer_json: &str, timeout: Duration) -> Result<String, String> {
    let url = format!("http://{ip}:{LEGACY_SIGNALING_PORT}/offer");
    let answer = post(
        &url,
        "application/json",
        Some(offer_json.to_string()),
        timeout,
    )
    .await?;
    answer_sdp(&answer)
}

fn answer_sdp(answer: &str) -> Result<String, String> {
    let answer: Value =
        serde_json::from_str(answer).map_err(|e| format!("answer isn't JSON: {e}"))?;
    let sdp = answer["sdp"].as_str().unwrap_or("").to_string();
    if sdp == "reject" {
        return Err("the Go2 is busy: another WebRTC client (the Unitree app, Go2 Ctrl, another run) holds it".into());
    }
    Ok(sdp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ecb_roundtrip_and_path_ending() {
        let key = "0123456789abcdef0123456789abcdef";
        assert_eq!(
            aes_ecb_decrypt(&aes_ecb_encrypt("hello go2", key), key).unwrap(),
            "hello go2"
        );
        // pairs "xA" "xB" "xC" "xD" "xJ" -> 0 1 2 3 9
        assert_eq!(path_ending("....xAxBxCxDxJ"), "01239");
    }

    #[test]
    fn data2_3_needs_the_key_the_robot_sealed_with() {
        let hex = |bytes: [u8; 16]| bytes.iter().map(|b| format!("{b:02x}")).collect::<String>();
        let right = hex(rand::random());
        let nonce: [u8; 12] = rand::random();
        let sealed = Aes128Gcm::new_from_slice(&parse_aes_key(&right).unwrap())
            .unwrap()
            .encrypt(Nonce::from_slice(&nonce), b"secret data1".as_slice())
            .unwrap();
        let (ciphertext, tag) = sealed.split_at(sealed.len() - 16);
        let data1 = B64.encode([ciphertext, &nonce, tag].concat());
        assert_eq!(decrypt_data1(&data1, 3, &right).unwrap(), "secret data1");
        assert!(decrypt_data1(&data1, 3, &hex(rand::random()))
            .unwrap_err()
            .contains("AES"));
        assert!(decrypt_data1(&data1, 3, "")
            .unwrap_err()
            .contains("aes_128_key"));
    }

    #[test]
    fn rsa_wraps_the_session_key_like_jsencrypt() {
        use rsa::pkcs8::EncodePublicKey;
        let private = rsa::RsaPrivateKey::new(&mut rand::thread_rng(), 1024).unwrap();
        let der = private.to_public_key().to_public_key_der().unwrap();
        let wrapped = rsa_encrypt(
            "0123456789abcdef0123456789abcdef",
            &B64.encode(der.as_bytes()),
        )
        .unwrap();
        let plain = private
            .decrypt(Pkcs1v15Encrypt, &B64.decode(wrapped).unwrap())
            .unwrap();
        assert_eq!(plain, b"0123456789abcdef0123456789abcdef");
    }

    #[test]
    fn offer_id_is_empty_on_the_dogs_hotspot() {
        assert_eq!(offer_id("192.168.12.1"), "");
        assert_eq!(offer_id("10.10.187.42"), "STA_localNetwork");
        assert_eq!(validation_reply("abc").len(), 24);
    }
}
