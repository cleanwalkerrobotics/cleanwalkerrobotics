# Company Orchestration

## Mission

Build and sell an intelligent robot product by securing upfront payment deals before committing significant capital. All physical manufacturing and hardware assembly is outsourced. The core team operates as an AI-orchestrated startup focused on design, coordination, and deal-making.

## Phase Overview

| Phase | Objective | Exit Criteria |
|-------|-----------|---------------|
| **1 — Cost Discovery** | Get professional cost estimates for hardware components + contract manufacturing | Have itemized cost model with ±15% confidence |
| **2 — Deal Packaging** | Build pitch materials, financial model, and outreach pipeline | Have polished pitch deck + target list ready |
| **3 — Deal Closing** | Secure upfront payment commitment from government or enterprise buyer | Signed LOI or contract with payment terms |
| **4 — Execution** | Engage suppliers and manufacturers, begin production | First prototype delivered |

**Current phase: 1 → 2 transition — Cost Discovery largely COMPLETE, Deal Packaging starting**

## Team Roster

| Role | File | One-liner |
|------|------|-----------|
| CEO | `ceo.md` | Orchestrates all agents, prioritizes work, makes final decisions |
| CTO | `cto.md` | Owns technical spec, validates all hardware/software architecture decisions |
| Procurement Lead | `procurement.md` | Sources hardware components, collects supplier quotes |
| Manufacturing Lead | `manufacturing.md` | Finds contract manufacturers, estimates production costs |
| Business Development | `biz-dev.md` | Identifies buyers, structures deals, manages customer pipeline |
| Finance | `finance.md` | Builds cost models, sets pricing, validates deal economics |
| Communications | `communications.md` | Builds website, creates outreach materials, manages professional presence |

## Communication Matrix

```
              CEO   CTO   PROC  MFG   BIZ   FIN   COMM
CEO            —     ✓     ✓     ✓     ✓     ✓     ✓
CTO           ✓      —     ✓     ✓     .     .     .
Procurement   ✓     ✓      —     ↔     .     ✓     .
Manufacturing ✓     ✓     ↔      —     .     ✓     .
BizDev        ✓     .     .     .      —     ✓     ✓
Finance       ✓     .     ✓     ✓     ✓      —     .
Communications✓     .     .     .     ✓     .      —

✓ = direct communication channel
↔ = tight collaboration (shared context)
. = through CEO only
```

**Key flows:**
- CTO → Procurement/Manufacturing: Technical specs and requirements flow down
- Procurement/Manufacturing → Finance: Cost data flows in
- Finance → BizDev: Pricing and margins flow out
- BizDev ↔ Communications: Pitch materials and website content
- CEO ↔ Everyone: Status, priorities, decisions

## Orchestration Loop

The CEO runs a continuous loop:

```
1. COLLECT    — Gather status updates from all agents
2. ASSESS     — Identify blockers, gaps, new information
3. PRIORITIZE — Rank open tasks by phase-goal impact
4. DELEGATE   — Assign tasks with clear deliverables and deadlines
5. REVIEW     — Check completed work against quality bar
6. DECIDE     — Make calls on escalated questions
→ repeat
```

## Decision Authority

| Decision Type | Decided By | Escalates To |
|--------------|------------|--------------|
| Technical architecture | CTO | CEO |
| Supplier selection (shortlist) | Procurement | CEO |
| Manufacturer selection (shortlist) | Manufacturing | CEO |
| Final vendor commitments | CEO | — |
| Pricing / deal terms | Finance + BizDev | CEO |
| Outreach messaging & tone | Communications | BizDev → CEO |
| Go/no-go on phase transition | CEO | — |
| Budget allocation | CEO | — |

## Context Management Rules

Each agent operates from its own role file. The orchestration file is shared context for the CEO only.

- **Role files** contain: mission, responsibilities, inputs/outputs, procedures, current objectives
- **Agents do not read other agents' role files** unless explicitly instructed by CEO
- **Handoffs** are structured: the sending agent produces a deliverable, CEO routes it to receiving agent with context summary
- **The CEO summarizes** cross-team context rather than sharing raw files — this keeps each agent's context window focused

## Shared Artifacts

Artifacts that multiple agents need access to are stored centrally and referenced by name:

- `docs/product-spec-research.md` — CTO reference: 50kg, IP65, ODRI Solo-12 design, safety certs $195-380K
- `docs/hardware-bom-research.md` — CTO/Procurement/Finance: Prototype $10.2K, production @1000 $5.5K, actuators 50-60% of BOM
- `docs/ml-compute-costs.md` — CTO: Training $0-80, inference $249 Jetson Orin Nano
- `docs/assembly-iteration-costs.md` — Manufacturing/Finance: 60hrs/unit, 3-5 iterations, $287K recommended path
- `docs/logistics-transport-costs.md` — Manufacturing/Finance: $64K for 10-unit pilot, $24.7K tariffs, $6.4K per robot
- `docs/certification-pilot-costs.md` — CTO/Finance: Austin min cert $18-43K, FCC SDoC $2.8-7K
- `docs/pilot-financial-model.md` — Finance/BizDev: Total $363K investment, $210K revenue @$3,500/mo, -$153K pilot P&L
- `docs/market-research.md` — BizDev: US-focused, no quadrupedal competitors, RaaS model preferred
- `docs/market-research-europe.md` — BizDev: Helsinki #1 EU, EUR 15-20B market, EU Machinery Reg 2023/1230
- `docs/market-research-asia-pacific.md` — BizDev: Singapore #1 APAC, zero tariffs, NEA grants
- `docs/market-research-mena-africa.md` — BizDev: Abu Dhabi already deploying autonomous sweepers, BEEAH partner
- `docs/market-research-americas.md` — BizDev: Toronto $50M, Calgary $7M, Chile zero tariffs, resort opportunity
- `docs/market-research-global-synthesis.md` — BizDev/CEO: Global TAM $3-5B, Veolia = #1 partner, IFAT = launch event
- `docs/ceo/strategy.md` — CEO: Master strategy with global market entry sequence
- `docs/ceo/decisions-log.md` — CEO: 9 ADRs recorded (digital-first, AGPL, Austin, pricing, tariffs, global entry, Veolia, IFAT)
- `docs/ceo/contacts.md` — BizDev/Procurement: Supplier and municipal contacts
- `docs/ceo/outreach-tracker.md` — BizDev: Email outreach log (empty — ready to fill)
- `apps/web/` — Communications: Marketing website LIVE at cleanwalkerrobotics.vercel.app
- `pitch-deck/` — Communications owns, BizDev uses (NOT YET CREATED)
- `deal-pipeline.md` — BizDev owns, CEO reads (NOT YET CREATED — use outreach-tracker.md for now)
