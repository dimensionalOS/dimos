"use client";

import { Loader2, MapPin, Plus, Search } from "lucide-react";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { rpcClient } from "@/lib/orpc";
import { cn } from "@/lib/utils";

// Rooms the robot has saved maps for (data/rooms.yaml on the robot). Edit to
// match your robot's registry — Tier A drives everything through the existing
// natural-language command pipe, so this list is just what we offer as buttons.
const KNOWN_ROOMS = ["kitchen", "office"];

// Tier A: no structured room API yet. Each control sends a natural-language
// instruction through the existing `commands.send` pipe; the robot's agent runs
// the matching skill (set_room / guess_room / save_map) and narrates the result
// back through the normal chat/voice channel.
export function RoomsPanel() {
  const [rooms, setRooms] = useState<string[]>(KNOWN_ROOMS);
  const [active, setActive] = useState<string | null>(null);
  const [newName, setNewName] = useState("");
  const [busy, setBusy] = useState<string | null>(null);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const send = async (key: string, text: string, note: string) => {
    setBusy(key);
    setError(null);
    setMsg(null);
    try {
      await rpcClient.commands.send({ text });
      setMsg(note);
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed to send command");
    } finally {
      setBusy(null);
    }
  };

  const selectRoom = (room: string) => {
    setActive(room);
    send(
      `select:${room}`,
      `You are in the ${room}. Load that room's saved map.`,
      `Told the robot it's in the ${room} — relocalizing into that map.`,
    );
  };

  const recognize = () =>
    send(
      "recognize",
      "Look around and figure out which room you are in, then load that room's map.",
      "Asked the robot to recognize the room — watch the chat for its answer.",
    );

  const saveNewRoom = () => {
    const name = newName.trim();
    if (!name) return;
    if (!rooms.includes(name)) setRooms([...rooms, name]);
    setNewName("");
    setActive(name);
    send(
      `save:${name}`,
      `Save the current map as ${name}.`,
      `Asked the robot to save the current map as "${name}".`,
    );
  };

  return (
    <div className="flex flex-col gap-3 rounded-xl border bg-card p-4">
      <div className="flex items-center justify-between gap-2">
        <span className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
          Rooms
        </span>
        <Button disabled={busy !== null} onClick={recognize} size="sm" variant="outline">
          {busy === "recognize" ? (
            <Loader2 className="animate-spin" size={14} />
          ) : (
            <Search size={14} />
          )}
          Recognize room
        </Button>
      </div>

      <div className="flex flex-wrap gap-2">
        {rooms.map((room) => (
          <Button
            className={cn(active === room && "border-signal text-signal")}
            disabled={busy !== null}
            key={room}
            onClick={() => selectRoom(room)}
            size="sm"
            variant="outline"
          >
            {busy === `select:${room}` ? (
              <Loader2 className="animate-spin" size={14} />
            ) : (
              <MapPin size={14} />
            )}
            {room}
          </Button>
        ))}
      </div>

      <div className="flex flex-col gap-1.5">
        <span className="text-muted-foreground text-xs">
          Generate a new room from the current map
        </span>
        <div className="flex gap-2">
          <Input
            onChange={(e) => setNewName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && saveNewRoom()}
            placeholder="new room name (e.g. garage)"
            value={newName}
          />
          <Button disabled={busy !== null || !newName.trim()} onClick={saveNewRoom}>
            {busy?.startsWith("save:") ? (
              <Loader2 className="animate-spin" size={16} />
            ) : (
              <Plus size={16} />
            )}
            Save
          </Button>
        </div>
      </div>

      {msg ? <p className="text-signal text-xs">{msg}</p> : null}
      {error ? <p className="text-destructive text-xs">{error}</p> : null}
    </div>
  );
}
