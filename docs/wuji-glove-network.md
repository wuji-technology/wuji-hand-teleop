# Wuji Glove networking & the multi-NIC gotcha

The Wuji Glove talks to the host over UDP on the harness LAN (factory default:
`192.168.1.100` = left, `192.168.1.101` = right, port `50001`). On a normal
single-NIC machine it just works. On harnesses that give **each glove receiver
its own network port**, there is a routing gotcha worth knowing.

## Symptom

- `Scan SNs` / the SDK **discovers** the glove, but the controller logs
  `wuji_sdk connect attempt #… failed: Connection timeout` and never streams.
- `ping 192.168.1.100` **fails**, yet the glove is powered and cabled.
- The Tianji arm (also on `192.168.1.x`) connects fine — so it is *not* a
  firewall or whole-network outage.

## Cause: multiple NICs on the same subnet (multi-homing)

If the host has several NICs all addressed on `192.168.1.0/24` (e.g. `.50`,
`.120`, `.121`), the kernel sends **unicast** to a glove out the single
lowest-metric route — one NIC — which may not be the NIC the glove sits behind.
The ARP request goes out the wrong port, gets no reply, and the connect times
out. **Discovery still works** because it uses UDP *broadcast*, which leaves
every NIC.

```
/proc/net/arp  ->  192.168.1.100 ... 00:00:00:00:00:00 ...   # never resolved
ping -I enp18s0f0 192.168.1.100  ->  OK                       # but reachable here!
ping       192.168.1.100         ->  fail                     # default route picks the wrong NIC
```

### The reboot trap

The two ports of a dual-port NIC (`enpXsYf0` / `enpXsYf1`) can be enumerated in
a **different order on each boot**, so which port reaches which glove can swap.
Any route that pins a glove to a hardcoded interface *name* is therefore correct
only half the time. **Never `ip route add … dev enpXsYfN` by name as a permanent
fix** — after the next reboot it may point at the wrong port and even shadow a
correct route.

## Diagnosis checklist

| Check | Command | Meaning |
| --- | --- | --- |
| NICs / IPs | `hostname -I` | several IPs on `192.168.1.x` ⇒ multi-homed |
| Same-subnet routes | `ip route \| grep 192.168.1.0/24` | one route per NIC ⇒ multi-homed |
| ARP resolved? | `cat /proc/net/arp \| grep 192.168.1.10` | all-zero MAC ⇒ wrong NIC |
| Which NIC reaches it | `for n in <nics>; do ping -c1 -I $n 192.168.1.100; done` | the one that replies |

## Fix

The container does this automatically: `docker/scripts/setup_glove_routes.sh`
runs from `entrypoint.sh` on every start. For each glove IP it leaves the
default route alone if the glove is already reachable, otherwise it **probes
each NIC and pins a `/32` route to the one that actually reaches the glove**.
Because it re-probes every boot, it adapts to port-name swaps; on single-NIC
machines it is a no-op.

Re-run it any time (e.g. after powering the gloves on after the container
started):

```bash
docker exec wuji-hand-teleop bash /entrypoint-scripts/setup_glove_routes.sh
```

It needs `NET_ADMIN`, which `docker-compose.yml` grants via `cap_add`.

### Manual / persistent alternatives

- **Pin port names by MAC** (most robust on a fixed machine): a
  `systemd-networkd` `.link` file matching each port's MAC to a stable name
  (e.g. `glove-left`), then route by that name — immune to enumeration order.
- **MAC-bound static routes** in NetworkManager: NM binds a connection to a
  MAC, so its static route follows the physical port across reboots.

Both require per-machine MAC values, which is why the shipped default is the
name-agnostic probe script above.
