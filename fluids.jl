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
end

# -----------------------------------------------------------------------------
#                           Initialization
# -----------------------------------------------------------------------------
function initialize_particles()

    particles = Vector{Particle}(undef, num_particles)
    
    for i in 1:num_particles
        pos = [rand() * box_size, rand() * box_size, rand() * 0.5]  # Only bottom half
        particles[i] = Particle(
            pos,
            [0.0, 0.0, 0.0],    # velocity
            [0.0, 0.0, -10.0],  # acceleration
            1000.0,             # density
            0.0,                # pressure
            1.0                 # mass
        )
    end
    
    return particles
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
#                       Density and Pressure Calculation
# -----------------------------------------------------------------------------
function calculate_density_pressure!(particles)

    for i in 1:length(particles)
        particles[i].density = 0.0
        
        # Calculate density
        for j in 1:length(particles)
            r_vec = particles[i].position - particles[j].position
            r = norm(r_vec)
            
            particles[i].density += particles[j].mass * kernel(r, smoothing_length)
        end
        
        # Calculate pressure 
        particles[i].pressure = stiff_coef * ((particles[i].density/target_density)^7 - 1)
    end
end


# -----------------------------------------------------------------------------
#                       Force Calculation 
# -----------------------------------------------------------------------------
function calculate_forces!(particles)
    
    for i in 1:length(particles)
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
        Fi_viscosity = particles[i].mass * viscosity_coef * laplacian_velocity
        
        # Gravity 
        Fi_gravity = particles[i].mass * [0.0, 0.0, -10.0]
        
        # Total forces
        Fi = Fi_pressure + Fi_viscosity + Fi_gravity
        
        # Update velocity and position
        particles[i].velocity .+= (Fi / particles[i].mass) .* dt
        particles[i].position .+= particles[i].velocity .* dt

        #@printf "Particle %d: Pos=(%.3f, %.3f, %.3f) Vel=(%.3f, %.3f, %.3f) Density=%.2f Pressure=%.2f Fi_pressure=(%.3f, %.3f, %.3f) Fi_viscosity=(%.3f, %.3f, %.3f)\n" i particles[i].position[1] particles[i].position[2] particles[i].position[3] particles[i].velocity[1] particles[i].velocity[2] particles[i].velocity[3] particles[i].density particles[i].pressure Fi_pressure[1] Fi_pressure[2] Fi_pressure[3] Fi_viscosity[1] Fi_viscosity[2] Fi_viscosity[3]
    end
end

# -----------------------------------------------------------------------------
#                       Boundary Conditions
# -----------------------------------------------------------------------------
function apply_boundary_conditions!(particles)
    
    for p in particles
        # X boundaries
        if p.position[1] < 0.0
            p.position[1] = 0.0
            p.velocity .*= -damping  
        elseif p.position[1] > box_size
            p.position[1] = box_size
            p.velocity .*= -damping  
        end
        
        # Y boundaries
        if p.position[2] < 0.0
            p.position[2] = 0.0
            p.velocity .*= -damping  
        elseif p.position[2] > box_size
            p.position[2] = box_size
            p.velocity .*= -damping  
        end
        
        # Z boundaries 
        if p.position[3] < 0.0
            p.position[3] = 0.0
            p.velocity .*= -damping  
        elseif p.position[3] > box_size
            p.position[3] = box_size
            p.velocity .*= -damping  
        end
    end
end


# -----------------------------------------------------------------------------
#                       Main Simulation Step
# -----------------------------------------------------------------------------
function simulate_step!(particles)
    calculate_density_pressure!(particles)
    calculate_forces!(particles)
    apply_boundary_conditions!(particles)
end

# -----------------------------------------------------------------------------
#                       Visualization
# -----------------------------------------------------------------------------
function visualize_sph(particles, step)
    x = [p.position[1] for p in particles]
    y = [p.position[2] for p in particles]
    z = [p.position[3] for p in particles]
    
    plt = scatter3d(x, y, z,
            markersize=3,
            markercolor=:blue,
            xlim=(0, box_size),
            ylim=(0, box_size),
            zlim=(0, box_size),
            title="Time $(round(step, digits=2))s",
            xlabel="X", ylabel="Y", zlabel="Z",
            legend=false,
            camera=(30, 30),
            size=(500, 600)
    )
    return plt
end

# -----------------------------------------------------------------------------
#                           SPH Parameters
# -----------------------------------------------------------------------------
const num_particles = 1000
const dt = 0.01
const box_size = 1.0
const damping = 0.8

const smoothing_length = 0.1
const stiff_coef = 100.0
const target_density = 1000.0
const viscosity_coef = 0.2

# -----------------------------------------------------------------------------
#                       Main Simulation
# -----------------------------------------------------------------------------
function main()

    particles = initialize_particles()
    
    t = 0.0
    frame_count = 0
    
    while t < 10.0
        simulate_step!(particles)
        
        if frame_count % 1 == 0  # Save every frame
            plt = visualize_sph(particles, t)
            display(plt)
        end
        
        t += dt
        frame_count += 1
    end
end

main()