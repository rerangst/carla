// AgExec: console lines into a running sim.
//
//   UnrealEditor ... -AgExecPort=8991
//
// Opens a UDP socket on 127.0.0.1 only. Every datagram is one console line
// (a cvar such as "ag.Splat.LodPixels 1", a console command such as
// "ag.Splat.Stats"), run on the game thread exactly as the in-game console
// would run it; whatever it printed goes back to the sender as one datagram,
// empty when the command said nothing. Off without the flag.
#pragma once

namespace AgExec
{
/// Call once the engine exists (OnPostEngineInit): binds the port if the flag is set.
void Start();
void Stop();
} // namespace AgExec
