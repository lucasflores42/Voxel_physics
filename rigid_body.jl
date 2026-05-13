using Plots, LinearAlgebra, Printf

# -----------------------------------------------------------------------------
#                                   Struct
# -----------------------------------------------------------------------------
mutable struct Particle
    position::Vector{Float64}
    velocity::Vector{Float64}
    acceleration::Vector{Float64}
    mass::Float64
    material::String
    radius::Float64
    rigidbody::Int64  
end

mutable struct RigidBody
    id::Int
    particle_indices::Vector{Int}
    cm::Vector{Float64}
    V::Vector{Float64}
    ω::Vector{Float64}
end

# -----------------------------------------------------------------------------
#                           Initialization
# -----------------------------------------------------------------------------
function create_cube!(particles, rigidbodies, id, offset, v_init, ω_init)

    particle_radius = 0.4
    particle_diam = 2 * particle_radius

    positions = [
        [0,0,0],[particle_diam,0,0],[particle_diam,particle_diam,0],[0,particle_diam,0],
        [0,0,particle_diam],[particle_diam,0,particle_diam],[particle_diam,particle_diam,particle_diam],[0,particle_diam,particle_diam]
    ]

    indices = Int[]

    for pos in positions
        p = Particle(
            offset .+ pos,
            [0.0,0.0,0.0],
            [0.0,0.0,0.0],
            1.0,
            "solid",
            particle_radius,
            id
        )
        push!(particles, p)
        push!(indices, length(particles))
    end

    # CM
    cm = calculate_center_of_mass([particles[i] for i in indices])[1]

    r_list = Vector{Vector{Float64}}()

    # velocity
    for i in indices
        r = particles[i].position .- cm
        push!(r_list, copy(r))

        particles[i].velocity .= cross(ω_init, r)
    end

    push!(rigidbodies, RigidBody(
        id,
        indices,
        cm,
        v_init,  
        ω_init
    ))
end

function create_sphere!(particles, rigidbodies, id, offset, v_init, ω_init)

    # x2+y2=R2−z2
    particle_radius = 0.2
    particle_diam = 2 * particle_radius 
    sphere_radius = 1
    thickness = 0.01

    positions = []

    for z in -sphere_radius:particle_radius:sphere_radius

        for x in -sphere_radius:particle_radius:sphere_radius
            for y in -sphere_radius:particle_radius:sphere_radius

                if x^2 + y^2 + z^2 >= sphere_radius^2 - thickness && x^2 + y^2 + z^2 <= sphere_radius^2 + thickness
                    push!(positions, [x, y, z])
                end

            end
        end
    end

    indices = Int[]

    for pos in positions
        p = Particle(
            offset .+ pos,
            [0.0,0.0,0.0],
            [0.0,0.0,0.0],
            1.0,
            "solid",
            particle_radius,
            id
        )
        push!(particles, p)
        push!(indices, length(particles))
    end

    # CM
    cm = calculate_center_of_mass([particles[i] for i in indices])[1]

    r_list = Vector{Vector{Float64}}()

    # velocity
    for i in indices
        r = particles[i].position .- cm
        push!(r_list, copy(r))

        particles[i].velocity .= cross(ω_init, r)
    end

    push!(rigidbodies, RigidBody(
        id,
        indices,
        cm,
        v_init,  
        ω_init
    ))
end


function create_disk!(particles, rigidbodies, id, disk_radius, offset, v_init, ω_init)

    # x2+y2=R2−z2
    particle_radius = 0.2
    particle_diam = 2 * particle_radius 
    thickness = 0.01

    positions = []

    for z in -thickness:particle_radius:thickness
        for x in -disk_radius:particle_radius:disk_radius
            for y in -disk_radius:particle_radius:disk_radius

                if x^2 + y^2 + z^2 <= disk_radius^2 
                    push!(positions, [x, y, z])
                end

            end
        end
    end

    indices = Int[]

    for pos in positions
        p = Particle(
            offset .+ pos,
            [0.0,0.0,0.0],
            [0.0,0.0,0.0],
            1000.0,
            "solid",
            particle_radius,
            id
        )
        push!(particles, p)
        push!(indices, length(particles))
    end

    # CM
    cm = calculate_center_of_mass([particles[i] for i in indices])[1]

    r_list = Vector{Vector{Float64}}()

    # velocity
    for i in indices
        r = particles[i].position .- cm
        push!(r_list, copy(r))

        particles[i].velocity .= cross(ω_init, r)
    end

    push!(rigidbodies, RigidBody(
        id,
        indices,
        cm,
        v_init,  
        ω_init
    ))
end
# -----------------------------------------------------------------------------
#                       Force Calculation 
# -----------------------------------------------------------------------------
function calculate_gravity!(particle1, mass, particles)
    
    F_gravity = zeros(3)
    
    for j in 1:length(particles)
        if particle1 === particles[j]  
            continue
        end
        
        r_vec = particles[j].position - particle1.position
        r = norm(r_vec)
        
        if r > 0.0001  
            F_gravity += gravity_coef * particle1.mass * particles[j].mass * r_vec / (r ^ 3)
        end
    end
    
    #return F_gravity
    return mass * [0.0, 0.0, -10.0]  
end

# -----------------------------------------------------------------------------
#                               Rigid bodies
# -----------------------------------------------------------------------------
function update_rigidbody!(particles, rb)

    cm_old = copy(rb.cm)

    #=
    # angular momentum 
    L = zeros(3)

    for i in rb.particle_indices
        p = particles[i]
        r = p.position .- cm_old  
        L .+= p.mass .* cross(r, p.velocity)
    end

    I = calculate_inertia_tensor(particles, rb)
    rb.ω .= inv(I) * L
    =#

    # translation 
    a = [0.0, 0.0, -30.0]
    rb.V .+= a .* dt
    rb.cm .+= rb.V .* dt 

    # update particles
    for i in rb.particle_indices
        p = particles[i]

        # old relative position
        r = p.position .- cm_old

        # rotate
        r .+= cross(rb.ω, r) .* dt

        p.position .= rb.cm .+ r
        p.velocity .= rb.V .+ cross(rb.ω, r)
    end
end

function calculate_rigidbodies!(particles, rigidbodies)
    for rb in rigidbodies
        update_rigidbody!(particles, rb)
    end
end

function calculate_inertia_tensor(particles, rb)

    I_tensor = zeros(3,3)
    I3 = Matrix{Float64}(I, 3, 3)  # identity

    for i in rb.particle_indices
        
        p = particles[i]
        r = p.position .- rb.cm   

        r2 = dot(r, r)
        rrT = r * transpose(r)

        I_tensor .+= p.mass .* (r2 .* I3 .- rrT)
    end

    return I_tensor
end

function calculate_center_of_mass(particles)

    total_mass = 0.0
    cm_x = 0.0
    cm_y = 0.0
    cm_z = 0.0
    
    for p in particles
        total_mass += p.mass
        cm_x += p.mass * p.position[1]
        cm_y += p.mass * p.position[2]
        cm_z += p.mass * p.position[3]
    end
    
    return [cm_x / total_mass, cm_y / total_mass, cm_z / total_mass], total_mass
end
# -----------------------------------------------------------------------------
#                           Calculate collisions
# -----------------------------------------------------------------------------
function calculate_collision!(particle1,particle2, particles, rigidbodies)

    r_vec = particle1.position - particle2.position
    r = norm(r_vec)

    if r < particle1.radius + particle2.radius && r >= 0.0001

        x1 = particle1.position
        x2 = particle2.position
        v1 = particle1.velocity
        v2 = particle2.velocity
        m1 = particle1.mass
        m2 = particle2.mass

        normal = (x1 - x2) / r
        overlap = particle1.radius + particle2.radius - r
        total_mass = m1 + m2

        r = particle1.radius + particle2.radius
        dv1 = - (1 + collision_restitution_coefficient) * m2 / (m1 + m2) * dot(v1 - v2, x1 - x2) * (x1 - x2) / r^2
        dv2 = - (1 + collision_restitution_coefficient) * m1 / (m1 + m2) * dot(v2 - v1, x2 - x1) * (x2 - x1) / r^2
         
        if particle1.rigidbody != 0

            rb = rigidbodies[particle1.rigidbody]
            shift = overlap * normal * (m2 / (m1 + m2))
            rb.cm += shift

            for i in rb.particle_indices
                particles[i].position .+= shift
            end
            
            M = sum(particles[i].mass for i in rb.particle_indices)
            I = calculate_inertia_tensor(particles, rb)
            invI = inv(I)

            Δp = m1 * dv1
            r_rel = particle1.position .- rb.cm 

            # rigid body update
            rb.V += Δp / M
            rb.ω += invI * cross(r_rel, Δp)

        else
            particle1.position .+= overlap * normal * (m2 / total_mass)
            particle1.velocity .+= dv1
        end

        if particle2.rigidbody != 0

            rb = rigidbodies[particle2.rigidbody]
            shift = overlap * normal * (m1 / (m1 + m2))
            rb.cm -= shift

            for i in rb.particle_indices
                particles[i].position .-= shift
            end

            M = sum(particles[i].mass for i in rb.particle_indices)
            I = calculate_inertia_tensor(particles, rb)
            invI = inv(I)

            Δp = m2 * dv2
            r_rel = particle2.position .- rb.cm

            rb.V += Δp / M
            rb.ω += invI * cross(r_rel, Δp)

        else
            particle2.position .-= overlap * normal * (m1 / total_mass)
            particle2.velocity .+= dv2
        end
    end
end

function calculate_collisions!(particles,rigidbodies)
    for i in 1:length(particles)
        for j in i+1:length(particles)
            #if particles[i].material == "water" || particles[i].material != particles[j].material
                calculate_collision!(particles[i],particles[j], particles, rigidbodies)
            #end
        end
    end
end

# -----------------------------------------------------------------------------
#                       Boundary Conditions
# -----------------------------------------------------------------------------
function apply_boundary_conditions!(particles)

    for p in particles
        if p.rigidbody == 0   
            for d in 1:3
                if p.position[d] < p.radius
                    p.position[d] = p.radius
                    p.velocity[d] *= -damping
                elseif p.position[d] > box_size - p.radius
                    p.position[d] = box_size - p.radius
                    p.velocity[d] *= -damping
                end
            end
        end
    end
end

function apply_boundary_conditions_rigidbodies!(particles, rigidbodies)

    for rb in rigidbodies

        M = sum(particles[i].mass for i in rb.particle_indices)
        I = calculate_inertia_tensor(particles, rb)
        invI = inv(I)

        for i in rb.particle_indices
            p = particles[i]

            for d in 1:3

                # --------------------------------------------------
                # LOWER WALL
                # --------------------------------------------------
                if p.position[d] < p.radius

                    old_p = p.position[d]
                    new_p = p.radius
                    shift = new_p - old_p

                    # contact point BEFORE correction
                    r_rel = p.position .- rb.cm

                    # shift entire rigid body
                    for j in rb.particle_indices
                        particles[j].position[d] += shift
                    end
                    rb.cm[d] += shift

                    # normal direction
                    normal = zeros(3)
                    normal[d] = 1.0

                    # rigid velocity at contact
                    v_contact = rb.V .+ cross(rb.ω, r_rel)

                    vn = dot(v_contact, normal)

                    if vn < 0
                        dv = -(1 + damping) * vn * normal
                        Δp = p.mass .* dv

                        rb.V .+= Δp / M
                        rb.ω .+= invI * cross(r_rel, Δp)
                    end

                # --------------------------------------------------
                # UPPER WALL
                # --------------------------------------------------
                elseif p.position[d] > box_size - p.radius

                    old_p = p.position[d]
                    new_p = box_size - p.radius
                    shift = new_p - old_p

                    r_rel = p.position .- rb.cm

                    for j in rb.particle_indices
                        particles[j].position[d] += shift
                    end
                    rb.cm[d] += shift

                    normal = zeros(3)
                    normal[d] = -1.0

                    v_contact = rb.V .+ cross(rb.ω, r_rel)

                    vn = dot(v_contact, normal)

                    if vn < 0
                        dv = -(1 + damping) * vn * normal
                        Δp = p.mass .* dv

                        rb.V .+= Δp / M
                        rb.ω .+= invI * cross(r_rel, Δp)
                    end
                end
            end
        end
    end
end
# -----------------------------------------------------------------------------
#                       Main Simulation Step
# -----------------------------------------------------------------------------
function simulate_step!(particles, rigidbodies)

    calculate_collisions!(particles, rigidbodies)
    calculate_rigidbodies!(particles, rigidbodies)
    
    apply_boundary_conditions!(particles)
    apply_boundary_conditions_rigidbodies!(particles, rigidbodies)
end

# -----------------------------------------------------------------------------
#                       Visualization
# -----------------------------------------------------------------------------
function visualize(particles, step)
    x = [p.position[1] for p in particles]
    y = [p.position[2] for p in particles]
    z = [p.position[3] for p in particles]

    markersizes = [p.radius * 15 for p in particles]  

    plt = scatter3d(x, y, z,
            markersize=markersizes,  
            xlim=(0, box_size),
            ylim=(0, box_size),
            zlim=(0, box_size),
            title="Time $(round(step, digits=2))s",
            xlabel="Y", ylabel="X", zlabel="Z",
            legend=false,
            camera=(30, 30),
            size=(600, 700),
            alpha=1.
    )
    
    return plt
end

# -----------------------------------------------------------------------------
#                            Parameters
# -----------------------------------------------------------------------------
# world
const tmax = 100.0
const dt = 0.001
const box_size = 10.0
const damping = 0.0

const smoothing_length = 0.1

# collision
const collision_restitution_coefficient = 0.0
const gravity_coef = 0.1

# -----------------------------------------------------------------------------
#                           Main Simulation
# -----------------------------------------------------------------------------
function main()

    particles = Particle[]
    rigidbodies = RigidBody[]

    #create_cube!(particles, rigidbodies, 1, [5-0.8,5,8], [0,0,-20], [0,0.0,0.0])
    #create_cube!(particles, rigidbodies, 2, [5,5,3], [0,0,0], [0,0,0.0])
    #create_sphere!(particles, rigidbodies, 1, [5,5,7.5], [-10,0,0], [0,0,0])
    #create_sphere!(particles, rigidbodies, 2, [5,5,3], [5,0,0], [0,0,0])

    #create_disk!(particles, rigidbodies, 1, 4, [5,5,1], [0,0,0], [0,0,0])

    create_disk!(particles, rigidbodies, 1, 1, [5,5,3], [2,2,0], [-3,3,0])
    
    t = 0.0
    frame_count = 0
    save_interval = max(1, round(Int, 0.01 / dt))
    
    while t < tmax
        
        if frame_count % save_interval == 0  # Save every X frame
            plt=visualize(particles, t)
            display(plt)
            sleep(0.01)  
        end
        simulate_step!(particles, rigidbodies)
        t += dt
        frame_count += 1
    end
end

main()