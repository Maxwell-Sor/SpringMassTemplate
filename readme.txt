All required elements are completed

- Particles are initialized from vertices with no duplicates in world coordinates
- Springs are initialized without duplicates, each particle is connected to each other by a single spring. Rest length is based on initial particle distance.
- Ground plane initialized
- Penalty springs are initialized once on penetration. Attach point is the nearest point on the plane at the moment of penetration. Penalty springs have their own kd and ks properties
- Penalty springs are correctly updated and detached
- Vertices are correctly updated in "Update" in the correct coordinate system based on the particles. Particles are updated in "FixedUpdate"
- Particle-particle spring forces are calculated once per fixedupdate, and then the negative of that force is applied on the other particle to avoid recomputation
- Mesh bound and normals are updated
- Symplectic Euler integration is used. Future position is updated based on future velocity rather than current velocity
- Simulator loop updates particles using FixedUpdate
- Testcase uses correct values