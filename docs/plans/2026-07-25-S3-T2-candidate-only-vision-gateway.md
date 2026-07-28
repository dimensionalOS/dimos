# S3-T2 Candidate-only Vision Gateway Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 在不接入实时视频和不授权机器人运动的前提下，把已有候选帧验证器完善为可后台执行、可超时降级、可审计 provider metadata 的候选视觉网关。

**Architecture:** 复用 `CandidateEvidenceBundle`、`TargetVerification` 和
`PersistedCandidateLoader`，新增 provider-neutral `CandidateVisionGateway`。
网关一次只接收一个候选的 1–3 张显式选择帧，立即返回 job snapshot，并在 daemon
worker 中调用 provider；轮询在 deadline 后独立收敛为 `uncertain`，因此 provider
不会阻塞本地停止线程。S3-T1 尚未完成，本任务只在保存帧 replay 边界验收，未来
`ObservationBundle` 只需适配成同一个 `CandidateEvidenceBundle`。

**Tech Stack:** Python 3.12、Pydantic v2、threading、pytest、现有 OpenAI Responses
适配器与 VisualMemory replay。

---

## 边界和不变量

- 网关不连接 Go2、不调用 navigation、不发布 `cmd_vel`，也不把视觉 `yes` 当作
  移动许可。
- 每个候选只允许 1–3 个唯一 frame ID；不支持连续视频上传。
- 同一 candidate/payload 重试返回同一 job；同一 candidate 的不同 payload
  fail-closed。
- job metadata 只记录 provider、model、attempt、耗时、frame IDs 和 typed error；
  不持久化或返回 base64 图像。
- provider 未返回、抛错或超过 deadline 均产生 `uncertain`；晚到结果不能覆盖超时
  终态。
- 本任务不声称 S3-T1、S3-R1 或 Stage 3 已完成。

### Task 1：先锁定 gateway job contract

**Files:**

- Create: `dimos/perception/test_vision_gateway.py`
- Create: `dimos/perception/vision_gateway.py`

**Steps:**

1. 写失败测试：`submit()` 立即返回 `pending`，snapshot 不含 base64。
2. 写失败测试：正常 verifier 形成 `completed + yes/no/uncertain + bbox + confidence`
   和 provider metadata。
3. 写最小 Pydantic contract：`VisionJobState`、`ProviderMetadata`、
   `VisionJobSnapshot`。
4. 运行：

   ```bash
   .venv/bin/python -m pytest dimos/perception/test_vision_gateway.py -q
   ```

   预期：新增 contract 测试通过。

### Task 2：后台隔离、deadline 和幂等

**Files:**

- Modify: `dimos/perception/test_vision_gateway.py`
- Modify: `dimos/perception/vision_gateway.py`

**Steps:**

1. 写阻塞 verifier 测试，断言 `submit()` 不等待 provider。
2. 用可注入 monotonic clock 推进 deadline，断言 snapshot 收敛为
   `completed/uncertain/provider_timeout`。
3. 写晚到结果测试，断言 timeout terminal 不被覆盖。
4. 写相同候选幂等、不同 payload 冲突和 busy/inflight 限制测试。
5. 用 daemon thread、锁和 event 实现最小 job lifecycle。
6. 重跑 Task 1–2 测试。

### Task 3：保存帧 replay 与现有 verifier 回归

**Files:**

- Modify: `dimos/perception/test_vision_gateway.py`
- Modify only if required: `dimos/perception/target_verification.py`
- Modify only if required: `dimos/perception/offline_target_verification.py`

**Steps:**

1. 用临时 `VisualMemory` 保存三帧，只选择两帧构建 bundle。
2. 通过 fake provider 跑 gateway，断言只有两帧 ID 进入 metadata，结果不含图像字节。
3. 验证 provider 异常变成 typed uncertain，不把异常传播到控制线程。
4. 运行：

   ```bash
   .venv/bin/python -m pytest \
     dimos/perception/test_target_verification.py \
     dimos/perception/test_offline_target_verification.py \
     dimos/perception/test_vision_gateway.py -q
   ```

### Task 4：静态检查和文档

**Files:**

- Modify: `docs/plans/2026-07-25-go2-three-stage-robot-validation-plan.md`
- Modify: `docs/PROJECT_CONTEXT.md`
- Modify: this plan

**Steps:**

1. 把 S3-T2 标记为 replay 软件完成，同时保留 `S3-T1` 与真机 gate 未完成。
2. 记录本任务不接 API 密钥、不上传真实帧、不启动真机。
3. 运行：

   ```bash
   .venv/bin/ruff check \
     dimos/perception/vision_gateway.py \
     dimos/perception/test_vision_gateway.py
   git diff --check
   ```

4. 本工作区按当前仓库约束不创建 commit。

## 验收

1. 保存帧 replay 能产生结构化 `yes/no/uncertain`、normalized bbox、confidence 和
   provider metadata。
2. `submit()` 不等待 provider；deadline 后为 typed uncertain。
3. 最多上传三张显式选择帧，job/result JSON 不含图像内容。
4. 同一候选重试幂等，payload 冲突 fail-closed，晚到结果不覆盖 timeout。
5. 既有 offline verifier 测试和新增 gateway 测试全部通过。

## 实现结果（2026-07-25）

**Status:** `COMPLETE — SAVED-FRAME REPLAY ONLY`

- 新增 `CandidateVisionGateway`、immutable job/provider metadata contract 和
  bounded daemon worker。
- `submit()` 不等待 provider；deadline、provider exception 和 invalid result
  都收敛为 typed `uncertain`。
- timeout 后仍运行的 provider worker 继续占用 slot，避免通过反复超时创建无界后台
  调用；晚到结果不覆盖 timeout terminal。
- 同一 candidate/payload 幂等；candidate ID 重用不同证据触发 conflict；并发超过
  `max_inflight` fail-closed。
- snapshot 只包含 frame ID、provider/model、耗时和 typed error，不包含 base64
  或 data URL。
- 验证：目标验证、offline CLI 和 gateway 联合回归 20/20；Ruff 与
  `git diff --check` 通过。
- 未做：没有构造真实 provider client、没有读取 API key、没有上传真实图片、没有
  启动或移动 Go2。S3-T1 live `ObservationBundle` 与 S3-R1 仍未完成。
