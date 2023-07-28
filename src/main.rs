#![allow(non_upper_case_globals, non_snake_case, dead_code)]
// https://github.com/dimforge/sparkl/blob/master/src/dynamics/particle.rs#L29
// https://phatymah.medium.com/calculation-of-the-address-of-an-element-in-1d-2d-and-3d-array-6a296ad81d1e
/*
Address of A[i][j][k] = B + S*(R*C*(i − L.R.) + C*(j − L.C.) + (k − L.W.))
Address of A[i][j][k] = R*C*(i) + C*(j) + (k)
Address of A[i][j][k] = 64*(i) + 8*(j) + (k)
A = name of the array

i = block a subset of an element whose address is to be found

j = row subset of an element whose address is to be found

k = column subset of an element whose address is to be calculated

B = Base address i.e. address of the first element of arrays/address of an array

R = total number of rows in an array

C = total number of columns in an array

*/
// use bevy::diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin}iii;
use bevy::{prelude::*, diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin}};
// use bevy_inspector_egui::quick::WorldInspectorPlugin;
use rayon::prelude::*;

mod cam;

const grid_res: i32 = 64;
const num_cells: usize = (grid_res * grid_res) as usize;

const dt: f32 = 0.2;
const sim_iterations: i32 = (1./dt) as i32;

const gravity: f32 = -0.3;

const rest_density: f32 = 4.0;
const dynamic_viscosity: f32 = 0.1;

const eos_stiffness: f32 = 10.0;
const eos_power: f32 = 4.;

#[derive(Component, Clone, Copy)]
struct Particle {
    v: Vec2,    // velocity
    C: Mat2,     // affine momentum matrix
    m: f32,     // mass
}

#[derive(Component, Debug, Clone, Copy)]
struct Cell {
    // https://github.com/rust-lang/rust/issues/72353 
    v: Vec2,    // velocity Z 
    m: f32,     // mass
}

impl Cell {
    pub fn new() -> Self {
        return Cell { v:Vec2::ZERO, m: 0. }
    }

    pub fn zero(&mut self) {
        self.m = 0.;
        self.v = Vec2::ZERO;
    }
}

#[derive(Resource)]
struct Grid{g: Vec<Cell>}

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
        .insert_resource(Grid{g: vec![Cell::new(); num_cells]})
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
	let particle_material = materials.add(Color::rgb(0.0, 0.0, 1.).into());
    
    let multiplier = 2;
    let box_x = 32; 
    let box_y = 32;
    let sx = grid_res / 2;
    let sy = grid_res / 2;
    for i in (sx - box_x/2) * 2..(sx + box_x/2) * 2 {
        for j in (sy - box_y/2) * multiplier..(sy + box_y/2) * multiplier {
            commands.spawn(PbrBundle {
                mesh: particle_mesh.clone(),
                material: particle_material.clone(),
                transform: Transform::from_translation(Vec3::new(
                        i as f32/multiplier as f32, j as f32/multiplier as f32, 1.,
                        )),
                        ..Default::default()
            }).insert(Particle{v: Vec2::ZERO, C: Mat2::ZERO, m: 1.});

        }
    }
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
    query: Query<(&Particle, &Transform)>,
) {
    //.par_iter_mut()
    query.for_each(|(p, t)| {
        // What cell the particle is associated to
        let cell_idx = t.translation.floor();
        // How far away the particle is away from the cell
        let cell_diff = (t.translation - cell_idx) - 0.5;

        let weights = [
            0.5 * (0.5 - cell_diff).powf(2.),
            0.75 - (cell_diff).powf(2.),
            0.5 * (0.5 + cell_diff).powf(2.),
        ];

        for gx in 0..3 {
            for gy in 0..3 {
                let weight = weights[gx].x * weights[gy].y;

                let cell_x = Vec2::from([
                                        (cell_idx.x + gx as f32 - 1.).floor(), 
                                        (cell_idx.y + gy as f32- 1.).floor(),
                ]);
                let cell_dist = (cell_x - Vec2::from((t.translation.x, t.translation.y))) + 0.5;
                // TODO Make sure it's component multiplication, aka x*x y*y z*z
                let Q = p.C * cell_dist;

                let mass_contrib = weight * p.m;
                let cell_index = (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                // TODO This is the only section stopping full parrelelization, can't get lockfree access to grid, maybe atomics?
                // AtomicPtr
                grid.g[cell_index as usize].m += mass_contrib;
                grid.g[cell_index as usize].v += mass_contrib * (p.v + Vec2::from(Q));
            }
        }
    });
}

fn p2g2 (
    mut grid: ResMut<Grid>,
    query: Query<(&Particle, &Transform)>,
) {
    query.for_each(|(p, t)| {
        let cell_idx = t.translation.floor();
        let cell_diff = (t.translation - cell_idx) - 0.5;

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
                    let weight = weights[gx].x * weights[gy].y;
                    let cell_x = Vec2::from([
                        (cell_idx.x + gx as f32 - 1.).floor(), 
                        (cell_idx.y + gy as f32 - 1.).floor(),
                    ]);
                    let cell_index = (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                    density += grid.g[cell_index as usize].m * weight;
            }
        }
        let volume = p.m / density;
        let pressure = (-0.1_f32).max(eos_stiffness * (density / rest_density).powf(eos_power) - 1.);
        // ! THIS IS 100% WRONG FOR 3D PLEASE HELP
        let mut stress = Mat2::from_cols_array(&[
                                               -pressure, 0., 
                                               0., -pressure
        ]);
        let mut strain = p.C;

        let trace = strain.y_axis.x + strain.x_axis.y;
        strain.x_axis.y = trace;
        strain.y_axis.x = trace;

        let viscosity_term = dynamic_viscosity * strain;
        stress += viscosity_term;

        let eq_16_term_0 = -volume * 4. * stress * dt;

        for gx in 0..3 {
            for gy in 0..3 {
                let weight = weights[gx].x * weights[gy].y;

                let cell_x = Vec2::from([
                                        (cell_idx.x + gx as f32 - 1.).floor(), 
                                        (cell_idx.y + gy as f32 - 1.).floor(),
                ]);
                let cell_dist = (cell_x - Vec2::from((t.translation.x, t.translation.y))) + 0.5;
                let cell_index = (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;

                let momentum = eq_16_term_0 * weight * cell_dist;
                grid.g[cell_index as usize].v += Vec2::from(momentum);
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
            let x = i / grid_res as usize;
            let y = i % grid_res as usize;
            if (x < 2) || (x > (grid_res - 3) as usize) { cell.v.x = 0.}
            if (y < 2) || (y > (grid_res - 3) as usize) { cell.v.y = 0.}
        }
    }
}

fn g2p (
    grid: ResMut<Grid>,
    mut query: Query<(&mut Particle, &mut Transform)>,
) {
    query.for_each_mut(|(mut p, mut t)| {

        p.v = Vec2::ZERO;

        let cell_idx = t.translation.floor();
        let cell_diff = (t.translation - cell_idx) - 0.5;

        let weights = [
            0.5 * (0.5 - cell_diff).powf(2.),
            0.75 - (cell_diff).powf(2.),
            0.5 * (0.5 + cell_diff).powf(2.),
        ];

        // estimating particle volume by summing up neighbourhood's weighted mass contribution
        // MPM course, equation 152 
        let mut b: Mat2 = Mat2::ZERO;
        for gx in 0..3 {
            for gy in 0..3 {
                let weight = weights[gx].x * weights[gy].y;

                let cell_x = Vec2::from([
                                        (cell_idx.x + gx as f32 - 1.).floor(), 
                                        (cell_idx.y + gy as f32 - 1.).floor(),
                ]);
                let cell_index = (cell_x.x as usize * grid_res as usize) + cell_x.y as usize;
                let dist = (cell_x - Vec2::from((t.translation.x, t.translation.y))) + 0.5;
                let w_v = grid.g[cell_index as usize].v * weight;

                let term = Mat2::from_cols(w_v * dist.x, w_v * dist.y);

                b += term;

                p.v += w_v;
            }
        }
        p.C = b.mul_scalar(4.);
        t.translation.x += p.v.x * dt;
        t.translation.y += p.v.y * dt;
        
        t.translation.x = t.translation.x.clamp(1., grid_res as f32 - 2.);
        t.translation.y = t.translation.y.clamp(1., grid_res as f32 - 2.);

        let x_n = Vec2::from((t.translation.x, t.translation.y)) + p.v;
        let wall_min = 3.;
        let wall_max = grid_res as f32 - 2.;
        if x_n.x < wall_min{ p.v.x += wall_min - x_n.x};
        if x_n.x > wall_max{ p.v.x += wall_max - x_n.x};
        if x_n.y < wall_min{ p.v.y += wall_min - x_n.y};
        if x_n.y > wall_max{ p.v.y += wall_max - x_n.y};



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
