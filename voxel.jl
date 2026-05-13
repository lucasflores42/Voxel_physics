using Plots, LinearAlgebra, Printf

# -----------------------------------------------------------------------------
#                           SPH Particle Data Structure
# -----------------------------------------------------------------------------
mutable struct Particle
    position::Vector{Float64}
    velocity::Vector{Float64}
    acceleration::Vector{Float64}

    density::Float64
    pressure::Float64
    mass::Float64

    material::String
    radius::Float64
    temperature::Float64
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
function create_particle(particles, mass, radius, x_init, v_init)

    p = Particle(
                x_init,
                v_init,
                [0.0,0.0,0.0],
                1000.0,             # density
                0.0,                # pressure
                mass,              # mass
                "solid",
                radius,    # radius
                1,
                0
            )
        push!(particles, p)
end

function create_liquid(particles, num_particles)
    
    for i in 1:num_particles

        pos = [rand() * 0.5, rand() * box_size, rand() * box_size]  # Only bottom half

        p = Particle(
            pos,
            [0.0, 0.0, 0.0],    # velocity
            [0.0, 0.0, 0.0],  # acceleration
            1000.0,             # density
            0.0,                # pressure
            0.1,                # mass
            "liquid",             # material 
            0.1,
            1,
            0
        )
        push!(particles, p)
    end
    
end

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
            1000.0,             # density
            0.0,                # pressure
            1.0,                # mass
            "solid",
            particle_radius,
            1,
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
    particle_radius = 0.15
    particle_diam = 2 * particle_radius 
    sphere_radius = 0.5
    thickness = 0.1

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
            1000.0,             # density
            0.0,                # pressure
            10.0,                # mass
            "solid",
            particle_radius,
            1,
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
            1000.0,             # density
            0.0,                # pressure
            1.0,                # mass
            "liquid",
            particle_radius,
            1,
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
#                       SPH Kernels
# -----------------------------------------------------------------------------
function kernel(r, h)
    q = r / h
    if q <= 1.0
        return (1.0 - 1.5*q*q + 0.75*q*q*q) / (π * h^3)
    elseif q <= 2.0
        return 0.25 * (2.0 - q)^3 / (π * h^3)
    else
        return 0.0
    end
end

# -----------------------------------------------------------------------------
#                           Temperature Calculation
# -----------------------------------------------------------------------------
function calculate_temperature!(particle1,particles)
    
    if particle1.material == "liquid"

        neighborhood_temperature = 0.0
        neighbors = 0
        for j in 1:length(particles)
            if particle1.material == particles[j].material 
                
                r_vec = particle1.position - particles[j].position
                r = norm(r_vec)
                
                if r <= smoothing_length
                    neighborhood_temperature += (norm(particles[j].velocity)^2)
                    neighbors += 1
                end
            end
        end

        # 15 is a good number
        particle1.temperature = 20 * neighborhood_temperature / neighbors
    end

    if particle1.material == "air"
        particle1.temperature += 10 * randn()
        #@printf "temperature: %.2f\n" particle1.temperature
    end

    #=
    # evaporation
    critical_temperature = 273.0
    if particle1.temperature > critical_temperature
        particle1.material = "air"
        #@printf "temperature: %.2f\n" particle1.temperature
    end
    # condensation
    if particle1.temperature <= critical_temperature
        particle1.material = "liquid"
        #@printf "temperature: %.2f\n" particle1.temperature
    end
    =#
end

# -----------------------------------------------------------------------------
#                       Density and Pressure Calculation
# -----------------------------------------------------------------------------
function calculate_density_pressure!(particle1,particles)

    particle1.density = 0.0
    
    # Calculate density
    for j in 1:length(particles)
        r_vec = particle1.position - particles[j].position
        r = norm(r_vec)
        
        particle1.density += particles[j].mass * kernel(r, smoothing_length)
    end
    
    # Calculate pressure 
    if particle1.material == "air"
        particle1.pressure = air_stiff_coef * ((particle1.density/air_target_density)^7 - 1)
    elseif particle1.material == "liquid"
        particle1.pressure = liquid_stiff_coef * ((particle1.density/liquid_target_density)^7 - 1)
    end
end


# -----------------------------------------------------------------------------
#                       Force Calculation 
# -----------------------------------------------------------------------------
function calculate_gravity(position, mass, id, particles)

    F_gravity = zeros(3)
    
    for j in 1:length(particles)
              
        if particles[j].material == "solid" #&& particles[j].rigidbody != id
            r_vec = particles[j].position - position
            r = norm(r_vec)
            
            if r > 0.0001  
                F_gravity += gravity_coef * mass * particles[j].mass * r_vec / (r ^ 3)
            end
        end
    end
    
    return F_gravity
    #return mass * [0.0, 0.0, 0]  
end

function liquid_dynamics!(particles)
    for i in 1:length(particles)
        calculate_temperature!(particles[i],particles)

        if particles[i].material == "liquid"

            calculate_density_pressure!(particles[i],particles)

            grad_pressure = zeros(3)
            laplacian_velocity = zeros(3)
            
            # Calculate pressure gradient and velocity Laplacian
            for j in 1:length(particles)
                if i == j
                    continue
                end
                
                r_vec = particles[i].position - particles[j].position
                r = norm(r_vec)
                
                if r > smoothing_length || r == 0
                    continue
                end
                
                # Kernel gradient calculation
                q = r / smoothing_length
                kernel_grad = zeros(3)
                if q <= 1.0
                    factor = (-3.0 + 2.25*q) / (π * smoothing_length^5)
                    kernel_grad = factor * r_vec
                elseif q <= 2.0
                    factor = -0.75 * (2.0 - q)^2 / (π * smoothing_length^5 * q)
                    kernel_grad = factor * r_vec
                end
                
                # Pressure gradient (Equation 6)
                pressure_term = (particles[i].pressure / (particles[i].density^2) + 
                            particles[j].pressure / (particles[j].density^2))
                grad_pressure += particles[j].mass * pressure_term * kernel_grad
                
                # Velocity Laplacian (Equation 8)
                v_ij = particles[i].velocity - particles[j].velocity
                dot_r_grad = dot(r_vec, kernel_grad)
                denominator = dot(r_vec, r_vec) + 0.01 * smoothing_length^2
                
                if denominator != 0
                    laplacian_velocity += 2.0 * (particles[j].mass / particles[j].density) * 
                                        v_ij * (dot_r_grad / denominator)
                end
            end
            
            # Pressure force (-∇P/ρ)
            Fi_pressure = -grad_pressure 
            
            # Viscosity force (ν∇²v)
            Fi_viscosity = particles[i].mass * liquid_viscosity_coef * laplacian_velocity
            
            # Gravity 
            Fi_gravity = calculate_gravity(particles[i].position, particles[i].mass, particles[i].rigidbody, particles)
            
            # Total forces
            Fi = Fi_pressure + Fi_viscosity + Fi_gravity
            
            # Update velocity and position
            particles[i].velocity .+= (Fi / particles[i].mass) .* dt
            particles[i].position .+= particles[i].velocity .* dt

            #@printf "Particle %d: Pos=(%.3f, %.3f, %.3f) Vel=(%.3f, %.3f, %.3f) Density=%.2f Pressure=%.2f Fi_pressure=(%.3f, %.3f, %.3f) Fi_viscosity=(%.3f, %.3f, %.3f)\n" i particles[i].position[1] particles[i].position[2] particles[i].position[3] particles[i].velocity[1] particles[i].velocity[2] particles[i].velocity[3] particles[i].density particles[i].pressure Fi_pressure[1] Fi_pressure[2] Fi_pressure[3] Fi_viscosity[1] Fi_viscosity[2] Fi_viscosity[3]
        end
    end
end

function air_dynamics!(particles)
    for i in 1:length(particles)
        if particles[i].material == "air"

            calculate_density_pressure!(particles[i],particles)

            grad_pressure = zeros(3)
            laplacian_velocity = zeros(3)
            
            # Calculate pressure gradient and velocity Laplacian
            for j in 1:length(particles)
                if i == j
                    continue
                end
                
                r_vec = particles[i].position - particles[j].position
                r = norm(r_vec)
                
                if r > smoothing_length || r == 0
                    continue
                end
                
                # Kernel gradient calculation
                q = r / smoothing_length
                kernel_grad = zeros(3)
                if q <= 1.0
                    factor = (-3.0 + 2.25*q) / (π * smoothing_length^5)
                    kernel_grad = factor * r_vec
                elseif q <= 2.0
                    factor = -0.75 * (2.0 - q)^2 / (π * smoothing_length^5 * q)
                    kernel_grad = factor * r_vec
                end
                
                # Pressure gradient (Equation 6)
                pressure_term = (particles[i].pressure / (particles[i].density^2) + 
                            particles[j].pressure / (particles[j].density^2))
                grad_pressure += particles[j].mass * pressure_term * kernel_grad
                
                # Velocity Laplacian (Equation 8)
                v_ij = particles[i].velocity - particles[j].velocity
                dot_r_grad = dot(r_vec, kernel_grad)
                denominator = dot(r_vec, r_vec) + 0.01 * smoothing_length^2
                
                if denominator != 0
                    laplacian_velocity += 2.0 * (particles[j].mass / particles[j].density) * 
                                        v_ij * (dot_r_grad / denominator)
                end
            end
            
            # Pressure force (-∇P/ρ)
            Fi_pressure = -grad_pressure 
            
            # Viscosity force (ν∇²v)
            Fi_viscosity = particles[i].mass * air_viscosity_coef * laplacian_velocity
            
            # Gravity 
            Fi_gravity = [0,0,0] #calculate_gravity(particles[i], particles[i].mass, particles)

            # Keep air in a specific z 
            air_region = 7
            air_region_width = 1
            air_region_strength = 5
            F_extra = [0,0,0]
            if particles[i].position[3] < air_region - air_region_width
                F_extra = air_region_strength .* [0, 0, 1]
            elseif particles[i].position[3] > air_region + air_region_width
                F_extra = - air_region_strength .* [0, 0, 1] 
            else
                #F_extra *= 0.2
                particles[i].velocity[3] *= 0.8
            end
            
            # Total forces
            Fi = Fi_pressure + Fi_viscosity + Fi_gravity + F_extra
            
            # Update velocity and position
            particles[i].velocity .+= (Fi / particles[i].mass) .* dt
            particles[i].position .+= particles[i].velocity .* dt

            #@printf "Particle %d: Pos=(%.3f, %.3f, %.3f) Vel=(%.3f, %.3f, %.3f) Density=%.2f Pressure=%.2f Fi_pressure=(%.3f, %.3f, %.3f) Fi_viscosity=(%.3f, %.3f, %.3f)\n" i particles[i].position[1] particles[i].position[2] particles[i].position[3] particles[i].velocity[1] particles[i].velocity[2] particles[i].velocity[3] particles[i].density particles[i].pressure Fi_pressure[1] Fi_pressure[2] Fi_pressure[3] Fi_viscosity[1] Fi_viscosity[2] Fi_viscosity[3]
        end
    end
end

function sand_dynamics!(particles)
    for i in 1:length(particles)
        if particles[i].material == "sand"
            
            # Gravity 
            Fi_gravity = calculate_gravity(particles[i], particles)
            
            # Total forces 
            Fi = Fi_gravity

            particles[i].velocity .+= (Fi / particles[i].mass) .* dt
            particles[i].position .+= particles[i].velocity * dt
        end
    end
end

function solid_dynamics!(particles)

    for i in 1:length(particles)
        if particles[i].material == "solid" && particles[i].rigidbody == 0

            Fi_gravity = calculate_gravity(particles[i].position, particles[i].mass, particles[i].rigidbody, particles)
            
            # Update velocity and position
            particles[i].velocity .+= (Fi_gravity / particles[i].mass) .* dt
            particles[i].position .+= particles[i].velocity .* dt   
        end
    end
end

# -----------------------------------------------------------------------------
#                               Rigid bodies
# -----------------------------------------------------------------------------
function update_rigidbody!(particles, rb)

    cm_old = copy(rb.cm)

    # translation 
    M = sum(particles[i].mass for i in rb.particle_indices)
    F_gravity = calculate_gravity(rb.cm, M, rb.id, particles)
    a = F_gravity / M
    rb.V .+= a .* dt
    rb.cm .+= rb.V .* dt 

    # update particles
    for i in rb.particle_indices
        p = particles[i]

        # old relative position
        r = p.position .- cm_old

        ω = rb.ω
        θ = norm(ω) * dt

        if θ > 0
            k = ω / norm(ω)  # axis of rotation

            r = r*cos(θ) +
                cross(k, r)*sin(θ) +
                k * dot(k, r) * (1 - cos(θ))
        end
        

        # rotate
        #r .+= cross(rb.ω, r) .* dt

        p.position .= rb.cm .+ r
        p.velocity .= rb.V .+ cross(rb.ω, r)

        
    end

    @printf "%s %s\n" rb.cm rb.V
end

function calculate_rigidbodies!(particles, rigidbodies)
    for rb in rigidbodies
        update_rigidbody!(particles, rb)
    end
end

function attach_particles(particles, particle1, particle2)

    if particle1.rigidbody == 0 && particle2.rigidbody == 0
        particle1.rigidbody = total_num_rigid_bodies + 1
        particle2.rigidbody = total_num_rigid_bodies + 1
        total_num_rigid_bodies += 1
    elseif particle1.rigidbody != 0 && particle2.rigidbody == 0
        particle2.rigidbody = particle1.rigidbody
    elseif particle1.rigidbody == 0 && particle2.rigidbody != 0
        particle1.rigidbody = particle2.rigidbody
    end
end

function detach_particles(particles, particle1)
    
    particle1.rigidbody = 0
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
#                           Calculate colisions
# -----------------------------------------------------------------------------
function calculate_colision!(particle1,particle2, particles, rigidbodies)

    r_vec = particle1.position - particle2.position
    r = norm(r_vec)

    if r < particle1.radius + particle2.radius && r >= 0.0001

        if particle1.rigidbody != 0 && particle2.rigidbody != 0

            rb1 = rigidbodies[particle1.rigidbody]
            rb2 = rigidbodies[particle2.rigidbody]
            x1 = particle1.position
            x2 = particle2.position
            v1 = particle1.velocity
            v2 = particle2.velocity
            m1 = sum(particles[i].mass for i in rb1.particle_indices)
            m2 = sum(particles[i].mass for i in rb2.particle_indices)

            normal = (x1 - x2) / r
            overlap = particle1.radius + particle2.radius - r
            total_mass = m1 + m2

            r = particle1.radius + particle2.radius
            dv1 = - (1 + colision_restitution_coefficient) * m2 / (m1 + m2) * dot(v1 - v2, x1 - x2) * (x1 - x2) / r^2
            dv2 = - (1 + colision_restitution_coefficient) * m1 / (m1 + m2) * dot(v2 - v1, x2 - x1) * (x2 - x1) / r^2

            shift1 = overlap * normal * (m2 / (m1 + m2))
            shift2 = overlap * normal * (m1 / (m1 + m2))
            rb1.cm += shift1
            rb2.cm -= shift2

            for i in rb1.particle_indices
                particles[i].position .+= shift1
            end

            for i in rb2.particle_indices
                particles[i].position .-= shift2
            end
            
            I1 = calculate_inertia_tensor(particles, rb1)
            invI1 = inv(I1)

            Δp1 = m1 * dv1
            r1_rel = particle1.position .- rb1.cm 

            # rigid body update
            rb1.V += Δp1 / m1
            rb1.ω += invI1 * cross(r1_rel, Δp1)

            I2 = calculate_inertia_tensor(particles, rb2)
            invI2 = inv(I2)

            Δp2 = m2 * dv2
            r2_rel = particle2.position .- rb2.cm

            rb2.V += Δp2 / m2
            rb2.ω += invI2 * cross(r2_rel, Δp2)
            
            if norm(dv1) > 0.1
                #detach_particles(particles, particle1)
            end

            if norm(dv2) > 0.1
                #detach_particles(particles, particle2)
            end
           
        elseif particle1.rigidbody == 0 && particle2.rigidbody == 0

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
            dv1 = - (1 + colision_restitution_coefficient) * m2 / (m1 + m2) * dot(v1 - v2, x1 - x2) * (x1 - x2) / r^2
            dv2 = - (1 + colision_restitution_coefficient) * m1 / (m1 + m2) * dot(v2 - v1, x2 - x1) * (x2 - x1) / r^2
            
            particle1.position .+= overlap * normal * (m2 / total_mass)
            particle1.velocity .+= dv1

            particle2.position .-= overlap * normal * (m1 / total_mass)
            particle2.velocity .+= dv2

        elseif particle1.rigidbody != 0 && particle2.rigidbody == 0

            rb1 = rigidbodies[particle1.rigidbody]
            x1 = particle1.position
            x2 = particle2.position
            v1 = particle1.velocity
            v2 = particle2.velocity
            m1 = sum(particles[i].mass for i in rb1.particle_indices)
            m2 = particle2.mass

            normal = (x1 - x2) / r
            overlap = particle1.radius + particle2.radius - r
            total_mass = m1 + m2

            r = particle1.radius + particle2.radius
            dv1 = - (1 + colision_restitution_coefficient) * m2 / (m1 + m2) * dot(v1 - v2, x1 - x2) * (x1 - x2) / r^2
            dv2 = - (1 + colision_restitution_coefficient) * m1 / (m1 + m2) * dot(v2 - v1, x2 - x1) * (x2 - x1) / r^2

            shift = overlap * normal * (m2 / (m1 + m2))
            rb1.cm += shift

            for i in rb1.particle_indices
                particles[i].position .+= shift
            end
            
            I1 = calculate_inertia_tensor(particles, rb1)
            invI1 = inv(I1)

            Δp1 = m1 * dv1
            r1_rel = particle1.position .- rb1.cm 

            # rigid body update
            rb1.V += Δp1 / m1
            rb1.ω += invI1 * cross(r1_rel, Δp1)
            
            if norm(dv1) > 0.1
                #detach_particles(particles, particle1)
            end

            particle2.position .-= overlap * normal * (m1 / total_mass)
            particle2.velocity .+= dv2

        elseif particle1.rigidbody == 0 && particle2.rigidbody != 0

            rb2 = rigidbodies[particle2.rigidbody]
            x1 = particle1.position
            x2 = particle2.position
            v1 = particle1.velocity
            v2 = particle2.velocity
            m1 = particle1.mass
            m2 = sum(particles[i].mass for i in rb2.particle_indices)

            normal = (x1 - x2) / r
            overlap = particle1.radius + particle2.radius - r
            total_mass = m1 + m2

            r = particle1.radius + particle2.radius
            dv1 = - (1 + colision_restitution_coefficient) * m2 / (m1 + m2) * dot(v1 - v2, x1 - x2) * (x1 - x2) / r^2
            dv2 = - (1 + colision_restitution_coefficient) * m1 / (m1 + m2) * dot(v2 - v1, x2 - x1) * (x2 - x1) / r^2

            particle1.position .+= overlap * normal * (m2 / total_mass)
            particle1.velocity .+= dv1

            shift = overlap * normal * (m1 / (m1 + m2))
            rb2.cm -= shift

            for i in rb2.particle_indices
                particles[i].position .-= shift
            end
            
            I2 = calculate_inertia_tensor(particles, rb2)
            invI2 = inv(I2)

            Δp2 = m2 * dv2
            r2_rel = particle2.position .- rb2.cm 

            # rigid body update
            rb2.V += Δp2 / m2
            rb2.ω += invI2 * cross(r2_rel, Δp2)
            
            if norm(dv2) > 0.1
                #detach_particles(particles, particle2)
            end

            
        end
    end
end

function calculate_colisions!(particles, rigidbodies)
    for i in 1:length(particles)
        for j in i+1:length(particles)
            #if particles[i].material == "liquid" || particles[i].material != particles[j].material
                calculate_colision!(particles[i],particles[j],particles, rigidbodies)
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
    liquid_dynamics!(particles)
    air_dynamics!(particles)
    sand_dynamics!(particles)
    solid_dynamics!(particles)
    calculate_colisions!(particles, rigidbodies)
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

    markersizes = []  
    colors = []

    for p in particles
        if p.material == "solid" && p.rigidbody == 0

            color = :purple
            size = p.radius * 15

            push!(colors, color)
            push!(markersizes, size)

        elseif p.material == "solid" && p.rigidbody != 0

            color = :yellow
            size = p.radius * 15

            push!(colors, color)
            push!(markersizes, size)

        elseif p.material == "liquid"

            t = clamp(p.temperature / 100.0, 0.0, 1.0)
            color = RGB(t, 0.0, 1.0 - t)  #
            size = p.radius * 15

            push!(colors, color)
            push!(markersizes, size)
        end
    end

    plt = scatter3d(x, y, z,
            markersize=markersizes,  
            markercolor=colors,
            xlim=(0, box_size),
            ylim=(0, box_size),
            zlim=(0, box_size),
            title="Time $(round(step, digits=2))s",
            xlabel="X", ylabel="Y", zlabel="Z",
            legend=false,
            camera=(30, 30),
            size=(600, 700),
            alpha=1.
    )
    
    return plt
end

# -----------------------------------------------------------------------------
#                           SPH Parameters
# -----------------------------------------------------------------------------
# world
const tmax = 100.0
const dt = 0.01
const box_size = 10.0
const damping = 0.

const smoothing_length = 0.1

# liquid
const liquid_target_density = 1000.0
const liquid_stiff_coef = 10.
const liquid_viscosity_coef = 0.2

# air
const air_target_density = 1000.0
const air_stiff_coef = 100.
const air_viscosity_coef = 0.05

# colision
const colision_restitution_coefficient = 0.0
const gravity_coef = 0.1

# -----------------------------------------------------------------------------
#                       Main Simulation
# -----------------------------------------------------------------------------
function main()

    particles = Particle[]
    rigidbodies = RigidBody[]

    #create_particle(particles, 1000, 0.5, [3,5,5], [0,-4,0])
    create_particle(particles, 1000, 0.5, [5,5,5], [0,0,0])
    create_liquid(particles, 500)
    #create_cube!(particles, rigidbodies, 1, [5-0.8,5,8], [0,0,-20], [0,0.0,0.0])
    #create_cube!(particles, rigidbodies, 2, [5,5,3], [0,0,0], [0,0,0.0])
    #create_sphere!(particles, rigidbodies, 1, [5,5,5], [0,0,0], [0,0,0])
    #create_sphere!(particles, rigidbodies, 2, [7,5,5], [0,4,0], [0,0,0])
    #create_sphere!(particles, rigidbodies, 1, [5,5,5], [0,0,0], [10,0,0])

    t = 0.0
    frame_count = 0
    save_interval = max(1, round(Int, 0.01 / dt))
    
    while t < tmax
        
        if frame_count % save_interval == 0  # Save every X frame
            plt = visualize(particles, t)
            display(plt)
            #sleep(0.1)  
        end
        simulate_step!(particles, rigidbodies)
        t += dt
        frame_count += 1
    end
end

main()