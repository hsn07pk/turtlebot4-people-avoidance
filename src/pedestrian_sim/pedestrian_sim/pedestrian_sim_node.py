"""
pedestrian_sim_node.py

Spawns N cylindrical "people" in the running Gazebo world and drives them
with a bounded random walk.  The cylinders are collidable, so the TB4
RPLidar A1 sees them as real obstacles.

Each person = one Gazebo model with two cylinder links (leg pair):

    left_leg   at local offset  (0,  +0.125, 0.9)  radius 0.10 m
    right_leg  at local offset  (0,  -0.125, 0.9)  radius 0.10 m

Setting the model pose rotates both legs together, keeping the hip-width
separation perpendicular to the walking direction at all times.  This
produces the two-blob LiDAR pattern that the leg-detection stage looks for.

Parameters
----------
world_name       str    depot      Gazebo world name (check: gz service -l | grep create)
num_people       int    2          Number of pedestrians to spawn
speed            float  0.5        Walking speed (m/s)
turn_noise_std   float  0.4        Heading noise std-dev per step (rad)
update_hz        float  5.0        Pose update rate (Hz)
boundary_radius  float  3.5        Soft boundary from origin; people steer back beyond this (m)
"""
from __future__ import annotations

import math
import os
import random
import subprocess
import tempfile
from dataclasses import dataclass

import rclpy
from rclpy.node import Node


# ---------------------------------------------------------------------------
# SDF template — one model, two leg-cylinder links
# ---------------------------------------------------------------------------

_PERSON_SDF = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <link name="left_leg">
      <pose>0 0.125 0.9 0 0 0</pose>
      <collision name="col">
        <geometry>
          <cylinder><radius>0.10</radius><length>1.8</length></cylinder>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <cylinder><radius>0.10</radius><length>1.8</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.55 0.9 1</ambient>
          <diffuse>0.2 0.55 0.9 1</diffuse>
        </material>
      </visual>
    </link>
    <link name="right_leg">
      <pose>0 -0.125 0.9 0 0 0</pose>
      <collision name="col">
        <geometry>
          <cylinder><radius>0.10</radius><length>1.8</length></cylinder>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <cylinder><radius>0.10</radius><length>1.8</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.55 0.9 1</ambient>
          <diffuse>0.2 0.55 0.9 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# Person state
# ---------------------------------------------------------------------------

@dataclass
class Person:
    name: str
    x: float
    y: float
    theta: float      # current heading (rad)
    sdf_path: str = ''


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class PedestrianSimNode(Node):

    def __init__(self) -> None:
        super().__init__('pedestrian_sim')

        self.declare_parameter('world_name',      'warehouse')
        self.declare_parameter('num_people',      2)
        self.declare_parameter('speed',           0.5)
        self.declare_parameter('turn_noise_std',  0.4)
        self.declare_parameter('update_hz',       5.0)
        self.declare_parameter('boundary_radius', 3.5)

        world = self.get_parameter('world_name').value
        n     = self.get_parameter('num_people').value

        self.people: list[Person] = []
        for i in range(n):
            # Place people evenly around a 1.5 m circle at startup
            angle = 2.0 * math.pi * i / n
            p = Person(
                name=f'sim_person_{i}',
                x=1.5 * math.cos(angle),
                y=1.5 * math.sin(angle),
                theta=angle + math.pi / 2.0,   # initial heading: tangent to circle
            )
            self._write_sdf(p)
            self._spawn(p, world)
            self.people.append(p)

        dt = 1.0 / self.get_parameter('update_hz').value
        self.create_timer(dt, self._step)

        self.get_logger().info(
            f'PedestrianSim: {n} people spawned in world "{world}". '
            f'If spawn failed, check world name with: gz service -l | grep create'
        )

    # ── Spawn helpers ─────────────────────────────────────────────────────────

    def _write_sdf(self, person: Person) -> None:
        """Write per-person SDF to a temp file so gz service can reference it."""
        fd, path = tempfile.mkstemp(suffix='.sdf', prefix=f'{person.name}_')
        with os.fdopen(fd, 'w') as f:
            f.write(_PERSON_SDF.format(name=person.name))
        person.sdf_path = path

    def _spawn(self, person: Person, world: str) -> None:
        req = (
            f'sdf_filename: "{person.sdf_path}" '
            f'name: "{person.name}" '
            f'pose {{ position {{ x: {person.x:.3f} y: {person.y:.3f} z: 0.0 }} }}'
        )
        try:
            result = subprocess.run(
                ['gz', 'service',
                 '-s', f'/world/{world}/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000',
                 '--req', req],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode != 0:
                self.get_logger().warn(
                    f'Spawn of {person.name} failed: {result.stderr.strip()}'
                )
            else:
                self.get_logger().info(
                    f'Spawned {person.name} at ({person.x:.1f}, {person.y:.1f})'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'Spawn of {person.name} timed out')

    # ── Random-walk step ─────────────────────────────────────────────────────

    def _step(self) -> None:
        world    = self.get_parameter('world_name').value
        speed    = self.get_parameter('speed').value
        turn_std = self.get_parameter('turn_noise_std').value
        boundary = self.get_parameter('boundary_radius').value
        dt       = 1.0 / self.get_parameter('update_hz').value

        for p in self.people:
            # Random heading perturbation
            p.theta += random.gauss(0.0, turn_std)

            # Forward step
            p.x += speed * dt * math.cos(p.theta)
            p.y += speed * dt * math.sin(p.theta)

            # Soft boundary: steer back toward origin when too far
            if math.hypot(p.x, p.y) > boundary:
                p.theta = math.atan2(-p.y, -p.x) + random.gauss(0.0, 0.3)

            self._set_pose(p, world)

    def _set_pose(self, person: Person, world: str) -> None:
        qz = math.sin(person.theta / 2.0)
        qw = math.cos(person.theta / 2.0)
        req = (
            f'name: "{person.name}" '
            f'position {{ x: {person.x:.4f} y: {person.y:.4f} z: 0.0 }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.5f} w: {qw:.5f} }}'
        )
        # Fire-and-forget — don't block the timer callback
        subprocess.Popen(
            ['gz', 'service',
             '-s', f'/world/{world}/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '200',
             '--req', req],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PedestrianSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
