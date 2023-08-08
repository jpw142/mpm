#![allow(non_upper_case_globals, non_snake_case, dead_code)]

use std::ops::{Add, AddAssign};

// https://github.com/dimforge/sparkl/blob/master/src/dynamics/particle.rs#L29
// https://phatymah.medium.com/calculation-of-the-address-of-an-element-in-1d-2d-and-3d-array-6a296ad81d1e
// use bevy::diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin}iii;
use bevy::{prelude::*, diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin}, math::{Vec3A, Mat3A}};
// use bevy_inspector_egui::quick::WorldInspectorPlugin;
use rayon::prelude::*;
mod cam;


pub const grid_res: i32 = 32;
const num_cells: usize = (grid_res * grid_res * grid_res) as usize;

const dt: f32 = 0.4;
const sim_iterations: i32 = (1./dt) as i32;

const gravity: f32 = -0.3;

const rest_density: f32 = 4.0;
const dynamic_viscosity: f32 = 0.1;

const eos_stiffness: f32 = 10.0;
const eos_power: f32 = 4.;

#[derive(Component, Clone, Copy)]
struct Particle {
    x: Vec3A,
    v: Vec3A,    // velocity
    C: Mat3A,     // affine momentum matrix
    m: f32,     // mass
}

#[repr(C)]
#[derive(Component, Debug, Clone, Copy)]
struct Node {
    // https://github.com/rust-lang/rust/issues/72353 
    v: Vec3A,    // velocity Z 
    m: f32,     // mass
}

impl Node {
    pub fn new() -> Self {
        return Node { v:Vec3A::ZERO, m: 0. }
    }

    pub fn zero(&mut self) {
        self.m = 0.;
        self.v = Vec3A::ZERO;
    }
}

#[derive(Resource)]
struct Grid{g: Vec<Node>}



fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // World Inspector Menu
        //.add_plugin(WorldInspectorPlugin::new())
         // Framerate logging
        .add_plugins((
                LogDiagnosticsPlugin::default(), 
                FrameTimeDiagnosticsPlugin::default(),
                cam::PlayerPlugin,
                ))
        .insert_resource(Grid{g: vec![Node::new(); num_cells]})
        .insert_resource(ClearColor(Color::rgb(1., 1., 1.)))
        .add_systems(Startup, initialize)
        .add_systems(Update, (
                draw_grid,
                clear_grid,
                p2g1.after(clear_grid),
                p2g2.after(p2g1),
                update_grid.after(p2g2),
                g2p.after(update_grid),
                ))
        .run();
}

fn initialize(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let particle_mesh = meshes.add(Mesh::from(shape::UVSphere{radius: 0.25, ..default()}));
    let particle_material = materials.add(StandardMaterial{
        base_color: Color::rgb(0.537, 0.612, 0.941),
        unlit: false,
        ..default()
    }); 
    let multiplier = 2;
    let box_width = 16;
    let box_x = box_width; 
    let box_y = box_width;
    let box_z = box_width;

    let sx = grid_res / 2;
    let sy = grid_res / 2;
    let sz = grid_res / 2;
    for i in (sx - box_x/2) * 2..(sx + box_x/2) * 2 {
        for j in (sy - box_y/2) * multiplier..(sy + box_y/2) * multiplier {
            for k in (sz - box_z/2) * multiplier..(sz + box_z/2) * multiplier {
               commands.spawn(PbrBundle {
                   mesh: particle_mesh.clone(),
                   material: particle_material.clone(),
                   transform: Transform::from_translation(Vec3::new(
                           i as f32/multiplier as f32, j as f32/multiplier as f32, k as f32/multiplier as f32,
                           )),
                           ..Default::default()
               }).insert(Particle{
                   x: Vec3A::new( i as f32/multiplier as f32, j as f32/multiplier as f32, k as f32/multiplier as f32),
                   v: Vec3A::ZERO,
                   C: Mat3A::ZERO,
                   m: 1.
               });

               // commands.spawn(Particle{v: Vec3::ZERO, C: Mat3::ZERO, m: 1.})
               //     .insert(Transform::from_translation((Vec3::new(
               //             i as f32/multiplier as f32, j as f32/multiplier as f32, k as f32/multiplier as f32))));
            }
        }
    }

    commands.spawn(SpotLightBundle {
            transform: Transform::from_xyz(grid_res as f32 / 2., grid_res as f32 - 10., grid_res as f32 / 2.)
                .looking_at(Vec3::new(grid_res as f32 / 2., 0.0, grid_res as f32 / 2.), Vec3::X),
                spot_light: SpotLight {
                    intensity: 16000.0, // lumens - roughly a 100W non-halogen incandescent bulb
                    color: Color::WHITE,
                    shadows_enabled: true,
                    inner_angle: 120.,
                    outer_angle: 2.,
                    ..default()
                },
                ..default()
        });
}

fn clear_grid (
    mut grid: ResMut<Grid>, 
) {
    grid.g.par_iter_mut().for_each(|cell| {
        cell.zero();
    })
}

fn p2g1 (
    mut grid: ResMut<Grid>,
    query: Query<&Particle>,
) {
    //.par_iter_mut()
    query.for_each(|p| {
        // What cell the particle is associated to
        let cell_idx = p.x.floor();
        // How far away the particle is away from the cell
        let cell_diff = (p.x - cell_idx) - 0.5;

        let weights = [
            0.5 * (0.5 - cell_diff).powf(2.),
            0.75 - (cell_diff).powf(2.),
            0.5 * (0.5 + cell_diff).powf(2.),
        ];

        for gx in 0..3 {
            for gy in 0..3 {
                for gz in 0..3 {
                    let weight = weights[gx].x * weights[gy].y * weights[gz].z;

                    let cell_x = Vec3A::from([
                                            (cell_idx.x + gx as f32 - 1.).floor(), 
                                            (cell_idx.y + gy as f32 - 1.).floor(),
                                            (cell_idx.z + gz as f32 - 1.).floor(),
                    ]);
                    let cell_dist = (cell_x - p.x) + 0.5;
                    // TODO Make sure it's component multiplication, aka x*x y*y z*z
                    let Q = p.C * cell_dist;

                    let mass_contrib = weight * p.m;
                    let cell_index = (cell_x.z as usize * grid_res as usize * grid_res as usize) + (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                    // TODO This is the only section stopping full parrelelization, can't get lockfree access to grid, maybe atomics?
                    // AtomicPtr
                    grid.g[cell_index as usize].m += mass_contrib;
                    grid.g[cell_index as usize].v += mass_contrib * (p.v + Vec3A::from(Q));
                }
            }
        }
    });
}

fn p2g2 (
    mut grid: ResMut<Grid>,
    query: Query<&Particle>,
) {
    query.for_each(|p| {
        let cell_idx = p.x.floor();
        let cell_diff = (p.x - cell_idx) - 0.5;

        let weights = [
            0.5 * (0.5 - cell_diff).powf(2.),
            0.75 - (cell_diff).powf(2.),
            0.5 * (0.5 + cell_diff).powf(2.),
        ];
        // estimating particle volume by summing up neighbourhood's weighted mass contribution
        // MPM course, equation 152 
        let mut density: f32 = 0.;
        for gx in 0..3 {
            for gy in 0..3 {
                for gz in 0..3 {
                    let weight = weights[gx].x * weights[gy].y * weights[gz].z;
                    let cell_x = Vec3A::from([
                        (cell_idx.x + gx as f32 - 1.).floor(), 
                        (cell_idx.y + gy as f32 - 1.).floor(),
                        (cell_idx.z + gz as f32 - 1.).floor(),
                    ]);
                    let cell_index = (cell_x.z as usize * grid_res as usize * grid_res as usize) + (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                    density += grid.g[cell_index as usize].m * weight;
                }
            }
        }
        let volume = p.m / density;
        let pressure = (-0.1_f32).max(eos_stiffness * (density / rest_density).powf(eos_power) - 1.);
        // ! THIS IS 100% WRONG FOR 3D PLEASE HELP
        let mut stress = Mat3A::from_cols_array(&[
                                               -pressure, 0., 0., 
                                               0., -pressure, 0.,
                                               0., 0., -pressure,
        ]);
        let mut strain = p.C;

        let trace = strain.z_axis.x + strain.y_axis.y + strain.x_axis.x;
        strain.x_axis.z = trace;
        strain.y_axis.y = trace;
        strain.z_axis.x = trace;

        let viscosity_term = dynamic_viscosity * strain;
        stress += viscosity_term;

        let eq_16_term_0 = -volume * 4. * stress * dt;

        for gx in 0..3 {
            for gy in 0..3 {
                for gz in 0..3 {
                    let weight = weights[gx].x * weights[gy].y * weights[gz].z;

                    let cell_x = Vec3A::from([
                                            (cell_idx.x + gx as f32 - 1.).floor(), 
                                            (cell_idx.y + gy as f32 - 1.).floor(),
                                            (cell_idx.z + gz as f32 - 1.).floor(),
                    ]);
                    let cell_index = (cell_x.z as usize * grid_res as usize * grid_res as usize) + (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                    let cell_dist = (cell_x - p.x) + 0.5;

                    let momentum = eq_16_term_0 * weight * cell_dist;
                    grid.g[cell_index as usize].v += Vec3A::from(momentum);
                }
            }
        }
    });
}

fn update_grid (
    mut grid: ResMut<Grid>,
) {
    for c in &mut grid.g.iter_mut().enumerate() {
        let (i, cell) = c;
        if cell.m > 0. {
            cell.v /= cell.m;
            cell.v.y += dt * gravity;
            
            let z = i /(grid_res as usize * grid_res as usize);
            let x = (i % (grid_res * grid_res) as usize) / grid_res as usize;
            let y = i % grid_res as usize;
            if (x < 2) || (x > (grid_res - 3) as usize) { cell.v.x = 0.}
            if (y < 2) || (y > (grid_res - 3) as usize) { cell.v.y = 0.}
            if (z < 2) || (z > (grid_res - 3) as usize) { cell.v.z = 0.}
        }
    }
}

fn g2p (
    grid: ResMut<Grid>,
    mut query: Query<(&mut Particle, &mut Transform)>,
) {
    query.par_iter_mut().for_each_mut(|(mut p, mut t)| {

        p.v = Vec3A::ZERO;

        let cell_idx = p.x.floor();
        let cell_diff = (p.x - cell_idx) - 0.5;

        let weights = [
            0.5 * (0.5 - cell_diff).powf(2.),
            0.75 - (cell_diff).powf(2.),
            0.5 * (0.5 + cell_diff).powf(2.),
        ];

        // estimating particle volume by summing up neighbourhood's weighted mass contribution
        // MPM course, equation 152 
        let mut b: Mat3A = Mat3A::ZERO;
        for gx in 0..3 {
            for gy in 0..3 {
                for gz in 0..3 {
                    let weight = weights[gx].x * weights[gy].y * weights[gz].z;

                    let cell_x = Vec3A::from([
                                            (cell_idx.x + gx as f32 - 1.).floor(), 
                                            (cell_idx.y + gy as f32 - 1.).floor(),
                                            (cell_idx.z + gz as f32 - 1.).floor(),
                    ]);
                    let cell_index = (cell_x.z as usize * grid_res as usize * grid_res as usize) + (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                    let dist = (cell_x - p.x) + 0.5;
                    let w_v = grid.g[cell_index as usize].v * weight;

                    let term = Mat3A::from_cols(w_v * dist.x, w_v * dist.y, w_v * dist.z);

                    b += term;

                    p.v += w_v;
                }
            }
        }
        p.C = b.mul_scalar(4.);

        t.translation += Vec3::from(p.v) * dt;

        t.translation = t.translation.clamp(Vec3::splat(1.), Vec3::splat(grid_res as f32 - 2.));
        
        p.x = Vec3A::from(t.translation);

        let x_n = p.x + p.v;
        let wall_min = 3.;
        let wall_max = grid_res as f32 - 2.;
        if x_n.x < wall_min{ p.v.x += wall_min - x_n.x};
        if x_n.x > wall_max{ p.v.x += wall_max - x_n.x};
        if x_n.y < wall_min{ p.v.y += wall_min - x_n.y};
        if x_n.y > wall_max{ p.v.y += wall_max - x_n.y};
        if x_n.z < wall_min{ p.v.z += wall_min - x_n.z};
        if x_n.z > wall_max{ p.v.z += wall_max - x_n.z};
    });
}

fn draw_grid (
    mut gizmos: Gizmos,
) {
    for x in 0..grid_res {
        for y in 0..grid_res {
            gizmos.cuboid(
                Transform::from_translation(Vec3::from([x as f32,y as f32,0.5 as f32])).with_scale(Vec3::splat(0.1)),                    
                Color::WHITE,
                );
        }
    }
}


// par_iter_mut
// .get()
// .get_many_mut()
// .get_component()
// 
