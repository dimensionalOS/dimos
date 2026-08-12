package com.dimensional.mini4pro.command

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The interlock on its own. What it can reach through the whole inbound MAVLink surface is
 * `CommandDispatcherTest`'s job; this file pins the three properties
 * `docs/decisions/2026-07-25-m2-command-safety.md` §Q2 asks for, including the two that are
 * properties of the class's *shape* rather than of its behaviour.
 */
class CommandInterlockTest {

    @Test
    fun `a fresh interlock is off`() {
        val interlock = CommandInterlock()
        assertFalse("commands must be off at every process start", interlock.enabled)
        assertNull(interlock.enabledSinceMs)
    }

    @Test
    fun `there is no way to construct an interlock that starts enabled`() {
        // The strongest form of "defaults to off" is that "on" is not expressible at
        // construction: no constructor parameter can be mistaken for an initial state, so no
        // caller, test or otherwise, can produce a live interlock without calling enable().
        // Reflection rather than a comment because a future author adding
        // `CommandInterlock(enabled = true)` for convenience would otherwise not be stopped.
        CommandInterlock::class.java.constructors.forEach { constructor ->
            constructor.parameterTypes.forEach { type ->
                assertFalse(
                    "a boolean constructor parameter would let an interlock start enabled: " +
                        constructor.parameterTypes.joinToString(),
                    type == java.lang.Boolean.TYPE || type == java.lang.Boolean::class.java,
                )
            }
        }
    }

    @Test
    fun `nothing about an enable survives into a new instance`() {
        // Stands in for "not persisted": within one process the only carrier of state across
        // instances would be a static field or a file, and neither exists. The failure this
        // guards is the one the decision doc names — a phone that boots into "commands live"
        // because of a setting from last week.
        val first = CommandInterlock()
        first.enable()
        assertTrue(first.enabled)

        assertFalse("a new interlock must not inherit an enable", CommandInterlock().enabled)
    }

    @Test
    fun `the interlock holds no reference to anything that could store it`() {
        // Reads every declared field and asserts it is a primitive, a lambda, or one of the two
        // types this class legitimately owns. A SharedPreferences, a File or a Context appearing
        // here is what persistence would look like on arrival.
        val allowed = setOf(
            java.util.concurrent.atomic.AtomicBoolean::class.java,
            java.lang.Long::class.java,
            kotlin.jvm.functions.Function0::class.java,
            kotlin.jvm.functions.Function1::class.java,
        )
        CommandInterlock::class.java.declaredFields.forEach { field ->
            val type = field.type
            assertTrue(
                "unexpected field ${field.name}: ${type.name} — persistence would look like this",
                type.isPrimitive || type in allowed,
            )
        }
    }

    @Test
    fun `enable and disable report whether they changed anything`() {
        var clock = 1_000L
        val interlock = CommandInterlock(nowMs = { clock })

        assertTrue(interlock.enable())
        assertEquals(1_000L, interlock.enabledSinceMs)
        assertFalse("a second enable changes nothing", interlock.enable())
        assertEquals("and does not restamp the clock", 1_000L, interlock.enabledSinceMs)

        clock = 9_000L
        assertTrue(interlock.disable())
        assertNull(interlock.enabledSinceMs)
        assertFalse(interlock.disable())

        // Re-enabling is a fresh session, so the timestamp is the new one.
        assertTrue(interlock.enable())
        assertEquals(9_000L, interlock.enabledSinceMs)
    }

    @Test
    fun `set is the toggle the UI needs and nothing more`() {
        val interlock = CommandInterlock()
        interlock.set(true)
        assertTrue(interlock.enabled)
        interlock.set(false)
        assertFalse(interlock.enabled)
    }

    @Test
    fun `state changes are logged, because a forgotten enable is the expected failure`() {
        val lines = mutableListOf<String>()
        val interlock = CommandInterlock(log = { lines.add(it) })

        interlock.enable()
        interlock.enable() // no-op, and must not log a second time
        interlock.disable()

        assertEquals(2, lines.size)
        assertTrue(lines[0].contains("ENABLED"))
        assertTrue(lines[1].contains("DISABLED"))
    }
}
