using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

// Check this out we can require components be on a game object!
[RequireComponent(typeof(MeshFilter))]

public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring (think about this ... may not even be needed o_0
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // is thi sparticle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = false;            // To render or not to render


    /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)

     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/

    public Transform groundPlane;
    public bool handlePlaneCollisions;
    public float particleMass;
    public bool useGravity;
    public Vector3 gravity;

    Mesh mesh;
    BParticle[] particles;
    BPlane plane;
    int[] vertexToParticleMap;



    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        InitParticles();
        InitPlane();
    }



    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/

    public void InitParticles()
    {
        /*Debug.Log("START");
        Debug.Log(mesh.vertices.Length);
        foreach(Vector3 pos in mesh.vertices){
            Debug.Log(pos);
        }
        Debug.Log(mesh.vertices);
        */


        BParticle newParticle; 
        Vector3 worldVertexPos;
        BSpring newSpring;
        List<BSpring> particleSprings;

        Vector3[] uniqueVertices = mesh.vertices.Distinct().ToArray();
        particles = new BParticle[uniqueVertices.Length];
        vertexToParticleMap = mapVertices(mesh.vertices, uniqueVertices);


        foreach(int i in vertexToParticleMap){
            Debug.Log(i);
        }

        for(int i = 0; i < uniqueVertices.Length; i++)
        {
            worldVertexPos = transform.TransformPoint(uniqueVertices[i]);
            particleSprings = new List<BSpring>();
            
            for (int p = 0; p < i; p++)
            {
                newSpring = new BSpring()
                {
                    kd = defaultSpringKD,
                    ks = defaultSpringKS,
                    restLength = Vector3.Distance(worldVertexPos, particles[p].position),
                    attachedParticle = p
                };
                
                particleSprings.Add(newSpring);
                //Debug.Log("Particle: " + i + ", Attached to Particle: " + newSpring.attachedParticle + ", Rest Length: " + newSpring.restLength);
            }

            newParticle = new BParticle()
            {
                position = worldVertexPos,
                velocity = Vector3.zero,
                mass = particleMass,
                attachedToContact = false,
                attachedSprings = particleSprings
            }; 

            particles[i] = newParticle;
        }
    }

    public int[] mapVertices(Vector3[] vertices, Vector3[] uniqueVertices)
    {
        int[] map = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            for (int j = 0; j < uniqueVertices.Length; j++)
            {
                if (vertices[i] == uniqueVertices[j])
                {
                    map[i] = j; // Map original index 'i' to particle index 'j'
                    break;
                }
            }
        }
        return map;
    }

    public void InitPlane()
    {
        plane = new BPlane
        {
            position = groundPlane.position,
            normal = groundPlane.up
        };

        //Debug.Log("Position: " + plane.position + ", Normal: " + plane.normal);
    }

    

    public void FixedUpdate()
    {
        /*Vector3 forces;
        Vector3 acceleration;
        BParticle particle;

        Vector3 newPosition;
        Vector3 newVelocity;

        float fixedDeltaTime = Time.fixedDeltaTime;

        Debug.Log(fixedDeltaTime);*/

        for (int i = 0; i < particles.Length; i++){
            BParticle particle = particles[i];
            /*forces = CalculateForces(particle);
            acceleration = forces / particleMass * fixedDeltaTime;

            newVelocity = particle.velocity + acceleration;
            newPosition = particle.position + newVelocity;

            particle.velocity = newVelocity;
            particle.position = newPosition;

            Debug.Log("Forces: " + forces + ", Acceleration: " + acceleration + ", velocity: " + particle.velocity + ", pos: " + particle.position);
            */
            particle.position = new Vector3(0f, 1f, 0f);
            //Debug.Log(particle.position);
        }

    }

    public Vector3 CalculateForces(BParticle particle)
    {
        return gravity * particleMass;
    }

    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
        UpdateMesh();
        //This will work if you have a correctly made particles array
        /*
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces, Color.blue);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
            }
        }
        */
    
    }

    public void UpdateMesh()
    {
        /*
        right down far
        left down far
        right up far
        left up far
        right up near
        left up near
        right down near
        left down near

        right up far
        left up far
        right up near
        left up near
        right down near
        right down far
        left down far
        left down near
        */

        Vector3[] currentMeshVertices = new Vector3[mesh.vertices.Length];

        for (int i = 0; i < currentMeshVertices.Length; i++)
        {
            int particleIndex = vertexToParticleMap[i];
            currentMeshVertices[i] = particles[particleIndex].position;
        }

        mesh.vertices = currentMeshVertices;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
}
