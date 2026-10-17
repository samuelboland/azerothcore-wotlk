/*
 * Lab-only runtime switches for behaviour-identical optimizations, so each can be measured on and off on a
 * running server. Every switch selects between two code paths with the same result. Refreshed from the
 * config at startup and on `reload config` (World::LoadConfigSettings). Default: on.
 */
#ifndef ACORE_LAB_SWITCHES_H
#define ACORE_LAB_SWITCHES_H

#include "Config.h"
#include <atomic>

struct LabSwitches
{
    static inline std::atomic<bool> SplineFlag{true};      // Unit::UpdateSplineMovement early return
    static inline std::atomic<bool> AuraStateCache{true};  // health aura states recomputed only on change
    static inline std::atomic<bool> AuraShortcut{true};    // Aura::UpdateOwner shortcut for non-periodic auras
    static inline std::atomic<bool> Prefetch{true};        // cache hints in Map::UpdateNonPlayerObjects
    static inline std::atomic<bool> ThreatCheck{true};     // ThreatManager::Update checks the inline map
    static inline std::atomic<bool> ScriptIdSkip{true};    // no script registry lookup for script id 0
    static inline std::atomic<bool> OwnerLookup{true};     // Creature::Update owner lookup only when charmed

    static bool On(std::atomic<bool> const& s) { return s.load(std::memory_order_relaxed); }

    static void Refresh()
    {
        auto get = [](char const* key) { return sConfigMgr->GetOption<bool>(key, true, false); };
        SplineFlag = get("Lab.Opt.SplineFlag");
        AuraStateCache = get("Lab.Opt.AuraStateCache");
        AuraShortcut = get("Lab.Opt.AuraShortcut");
        Prefetch = get("Lab.Opt.Prefetch");
        ThreatCheck = get("Lab.Opt.ThreatCheck");
        ScriptIdSkip = get("Lab.Opt.ScriptIdSkip");
        OwnerLookup = get("Lab.Opt.OwnerLookup");
    }
};

#endif
