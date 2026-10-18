/*
 * Lab-only runtime switches for behaviour-identical optimizations, so each can be measured on and off on a
 * running server. Every switch selects between two code paths with the same result. Refreshed from the
 * config at startup and on `reload config` (World::LoadConfigSettings). Default: on.
 */
#ifndef ACORE_LAB_SWITCHES_H
#define ACORE_LAB_SWITCHES_H

#include "Config.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

struct LabSwitches
{
    static inline std::atomic<bool> SplineFlag{true};      // Unit::UpdateSplineMovement early return
    static inline std::atomic<bool> AuraStateCache{true};  // health aura states recomputed only on change
    static inline std::atomic<bool> AuraShortcut{true};    // Aura::UpdateOwner shortcut for non-periodic auras
    static inline std::atomic<bool> Prefetch{true};        // cache hints in Map::UpdateNonPlayerObjects
    static inline std::atomic<bool> ThreatCheck{true};     // ThreatManager::Update checks the inline map
    static inline std::atomic<bool> ScriptIdSkip{true};    // no script registry lookup for script id 0
    static inline std::atomic<bool> OwnerLookup{true};     // Creature::Update owner lookup only when charmed
    static inline std::atomic<bool> GoAiSkip{true};        // skip empty GameObjectAI::UpdateAI calls
    static inline std::atomic<bool> SeeHookSkip{true};     // skip CanSeeAlways/CanAlwaysBeDetectable while no AI overrides them
    static inline std::atomic<bool> SplineSegCache{true};  // MoveSpline caches the current segment's times, line and facing
    static inline std::atomic<bool> SmartSkip{true};       // SmartScript skips its event loop while no event can act
    static inline std::atomic<bool> MotionIdleSkip{true};  // Unit::Update skips UpdateMotion while the idle generator is on top

    // Verification (Lab.Verify = 1): where an optimized path is taken, the original code also runs and the results
    // are compared. Counts are per thread (no shared counter on hot paths) and summed when logged.
    enum VerifyId { V_SPLINE_POS, V_SPLINE_TIME, V_SMART_SKIP, V_SMART_TYPES, V_MOTION_IDLE, V_AURA_STATE, V_COUNT };
    static inline std::atomic<bool> Verify{false};
    static bool VerifyOn() { return Verify.load(std::memory_order_relaxed); }
    static char const* VerifyName(int id)
    {
        static char const* const names[V_COUNT] = { "spline position", "spline segment end", "smart script skip",
            "smart script event types", "motion idle skip", "health aura states" };
        return names[id];
    }
    struct VerifyCounters { std::atomic<uint64_t> checks[V_COUNT]{}; std::atomic<uint64_t> bad[V_COUNT]{}; };
    static std::vector<VerifyCounters*>& VerifyRegistry() { static std::vector<VerifyCounters*> r; return r; }
    static std::mutex& VerifyRegistryLock() { static std::mutex m; return m; }
    static VerifyCounters& MyVerifyCounters()
    {
        thread_local VerifyCounters* mine = []
        {
            VerifyCounters* c = new VerifyCounters();   // lives as long as the process, like the threads
            std::lock_guard<std::mutex> guard(VerifyRegistryLock());
            VerifyRegistry().push_back(c);
            return c;
        }();
        return *mine;
    }
    // Counts one comparison. Returns true for a mismatch whose details should be logged (the first 20 of each kind).
    static bool Check(VerifyId id, bool ok)
    {
        VerifyCounters& c = MyVerifyCounters();
        c.checks[id].store(c.checks[id].load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        if (ok)
            return false;
        c.bad[id].store(c.bad[id].load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        static std::atomic<uint32_t> logged[V_COUNT]{};
        return logged[id].fetch_add(1, std::memory_order_relaxed) < 20;
    }
    static void VerifyTotals(uint64_t (&checks)[V_COUNT], uint64_t (&bad)[V_COUNT])
    {
        for (int i = 0; i < V_COUNT; ++i)
            checks[i] = bad[i] = 0;
        std::lock_guard<std::mutex> guard(VerifyRegistryLock());
        for (VerifyCounters* c : VerifyRegistry())
            for (int i = 0; i < V_COUNT; ++i)
            {
                checks[i] += c->checks[i].load(std::memory_order_relaxed);
                bad[i] += c->bad[i].load(std::memory_order_relaxed);
            }
    }

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
        GoAiSkip = get("Lab.Opt.GoAiSkip");
        SeeHookSkip = get("Lab.Opt.SeeHookSkip");
        SplineSegCache = get("Lab.Opt.SplineSegCache");
        SmartSkip = get("Lab.Opt.SmartSkip");
        MotionIdleSkip = get("Lab.Opt.MotionIdleSkip");
        Verify = sConfigMgr->GetOption<bool>("Lab.Verify", false, false);
    }
};

#endif
