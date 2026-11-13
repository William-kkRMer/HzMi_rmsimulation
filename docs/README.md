# `docs/` —— 10 份文档（2026-11-13 定版）

> **这份就是全部。** 原来的 119 份按主题合并成 **8 份主题文档 + 本索引**；
> 合并时**原文全部保留**，每篇开头有目录、正文按 `## 来源：<原文件名>` 分节，可搜索到原始位置。
> **历史原文**只在 `archive/`（明确标了"别看了"）。

## 先看哪份

| 我想… | 看这份 | 行数 |
|---|---|---:|
| **跑起来 / 建图 / 验收 / 看 RViz / 三套模式 / 日志分析** | [`05-operations.md`](05-operations.md) | 2,594 |
| **遇到问题（车不动、定位跳、规划失败…）** | [`04-issues.md`](04-issues.md) | 9,222 |
| **改 TF / 帧 / 参数**（先读契约） | [`02-contracts.md`](02-contracts.md) | 2,955 |
| **想知道某条契约的实测依据** | [`03-measurements.md`](03-measurements.md) | 13,445 |
| **看懂了整体结构 / 槽位 / 名词** | [`01-architecture.md`](01-architecture.md) | 5,097 |
| **对照 T-DT（我们改了什么、差在哪）** | [`06-tdt.md`](06-tdt.md) | 8,147 |
| **改建图 / 重定位 / 地图资产** | [`07-mapping.md`](07-mapping.md) | 5,228 |
| **改仿真世界 / 后端 / 场地** | [`08-sim-worlds.md`](08-sim-worlds.md) | 5,097 |

## 八份里各有什么（合并来源）

| 文件 | 由这些合并而来 |
|---|---|
| `01-architecture.md` | architecture · glossary · algorithm_matrix · algorithm_axes · lio/localization/ground_segmentation slots · nav_stages · params_ownership_checklist · rm_algorithm_overview/catalog · roadmap_2026_11 · rm_algolab_plan · rm_bench_refactor_plan |
| `02-contracts.md` | frame_contract_v3 · frame_contract_v3_rationale · tf_interface_contract · sim_real_contract |
| `03-measurements.md` | frame_contract_v3_measurements · timestamp_construction_audit · live_map_chain_measurements · gravity_alignment_ownership_survey · cod_real_frame_measurement · odom_base_link_practice_survey · lio_extrinsic_application_survey |
| `04-issues.md` | issues_and_findings · path_clearance_and_contact · obstacle_judgement_map · recovery_fsm_plan · slope_speed_limiting · stvl_local_costmap · traversability_plan · kinodynamic_esdf_mpc_notes · nav2_vs_custom_planner · nav_strategy_taxonomy · gazebo_gui_troubleshooting · lio_drift_diagnosis · lio_divergence_no_impact · research_p2l_scan_stall · build_optimization · cloud_density_measurements · cartographer_2d_occupancy_semantics · livox_viewer2_launch_notes · livox_sensor_prior_art |
| `05-operations.md` | smoke_test_runbook · machine_hygiene_runbook · rviz_guide · gz_acceptance_checklist · runbook_2027_mapping · mapping/README · **§7 2026-11 新增操作面**（三套模式 / 地图流程 / 日志 / 诊断脚本 / UWB / BIT 前提）|
| `06-tdt.md` | tdt_chain_comparison（含已合并的 9 份对照）· tdt_code_study · tdt_component_map · tdt_planning_explained · tdt_full_stack · tdt_diff_inventory · tdt_integration_plan · tdt_integration_report · tdt_full_completion_status · cod_nav_*（7 份）· cod_vs_ours_frame_architecture |
| `07-mapping.md` | mapping_architecture · mapping_2d_from_cloud · 3d_to_2d_survey · mapping_relocalization_survey · relocalization_practice_survey · mapping_small_point_lio · scan_context_plan/probe/gate · map_assets · CARTOGRA_PBSTREAM_GUIDE · mapping_session_snapshots_merged |
| `08-sim-worlds.md` | gazebo_sim_migration_plan · worlds · worlds_2027 · round_summary_gz_frames · round_summary_hybrid_default · session_digest · system_status |

## 已删除（不再保留）

| 文档 | 原因 |
|---|---|
| `continue_mapping.md`（分段/续建建图） | 不是常见必要流程 |
| `inventory.md`（逐份清单） | 合并后已无逐份文件可列 |
| `learn/`（6 份教材，15,418 行） | 与项目操作无关 |

以上都能从 git 历史取回：`git log --all --diff-filter=D -- docs/`

## 目录

```
docs/
├── README.md               ← 本文件（索引）
├── 01-architecture.md
├── 02-contracts.md
├── 03-measurements.md
├── 04-issues.md
├── 05-operations.md
├── 06-tdt.md
├── 07-mapping.md
├── 08-sim-worlds.md
├── archive/                ← 历史原文，**不用看**
├── assets/  img/           ← 图片/配置证据
├── mapping/                ← 只剩一张快速参考卡片
└── package/                ← 配置包说明
```

## 维护约定

> **不再新增平行文档**。新内容直接追加到对应的 0X-*.md（`## 来源：<主题>` 下），
> 或在 `04-issues.md` 里加一条「现象 → 根因 → 修法」。
> 确实需要独立成篇时，先问一句"能不能并进这 8 份里"。
