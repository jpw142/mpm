// https://github.com/dimforge/sparkl/blob/4517c9f96acf2e7588b31dbab744a616608af2af/src/geometry/sp_grid.rs

use bevy::{math::{Vec3A, UVec3}};
use anyhow::Result;
use bevy::prelude::info;
use bytemuck::Pod;
use memmap2::MmapMut;
use std::marker::PhantomData;
// Generated by printing the result of `nbh_shift()`.

const PACKED_NBH_SHIFTS: [u64; 27] = [
    2688, 2176, 2432, 2560, 2048, 2304, 2624, 2112, 2368, 640, 128, 384, 512, 0, 256, 576, 64, 320,
    1664, 1152, 1408, 1536, 1024, 1280, 1600, 1088, 1344,
];

const PACKED_NBH_REGION_SHIFTS: [u64; 7] = [16384, 8192, 24576, 4096, 20480, 12288, 28672];

const NBH_SHIFTS: [UVec3; 27] = [
    UVec3::new(2, 2, 2),
    UVec3::new(2, 0, 2),
    UVec3::new(2, 1, 2),
    UVec3::new(0, 2, 2),
    UVec3::new(0, 0, 2),
    UVec3::new(0, 1, 2),
    UVec3::new(1, 2, 2),
    UVec3::new(1, 0, 2),
    UVec3::new(1, 1, 2),
    UVec3::new(2, 2, 0),
    UVec3::new(2, 0, 0),
    UVec3::new(2, 1, 0),
    UVec3::new(0, 2, 0),
    UVec3::new(0, 0, 0),
    UVec3::new(0, 1, 0),
    UVec3::new(1, 2, 0),
    UVec3::new(1, 0, 0),
    UVec3::new(1, 1, 0),
    UVec3::new(2, 2, 1),
    UVec3::new(2, 0, 1),
    UVec3::new(2, 1, 1),
    UVec3::new(0, 2, 1),
    UVec3::new(0, 0, 1),
    UVec3::new(0, 1, 1),
    UVec3::new(1, 2, 1),
    UVec3::new(1, 0, 1),
    UVec3::new(1, 1, 1),
];

#[allow(dead_code)] // Used only to generate the packed offset tables.
const NBH_REGION_SHIFTS: [[u64; 3]; 7] = [
    [0, 0, 4],
    [0, 4, 0],
    [0, 4, 4],
    [4, 0, 0],
    [4, 0, 4],
    [4, 4, 0],
    [4, 4, 4],
];


#[allow(dead_code)] // Used only to generate the packed offset tables.
fn nbh_region_shifts() -> [u64; 7] {
    [
        pack(0, 0, 4),
        pack(0, 4, 0),
        pack(0, 4, 4),
        pack(4, 0, 0),
        pack(4, 0, 4),
        pack(4, 4, 0),
        pack(4, 4, 4),
    ]
}

#[allow(dead_code)] // Used only to generate the packed offset tables.
fn nbh_shifts() -> [u64; 27] {
    [
        pack(2, 2, 2),
        pack(2, 0, 2),
        pack(2, 1, 2),
        pack(0, 2, 2),
        pack(0, 0, 2),
        pack(0, 1, 2),
        pack(1, 2, 2),
        pack(1, 0, 2),
        pack(1, 1, 2),
        pack(2, 2, 0),
        pack(2, 0, 0),
        pack(2, 1, 0),
        pack(0, 2, 0),
        pack(0, 0, 0),
        pack(0, 1, 0),
        pack(1, 2, 0),
        pack(1, 0, 0),
        pack(1, 1, 0),
        pack(2, 2, 1),
        pack(2, 0, 1),
        pack(2, 1, 1),
        pack(0, 2, 1),
        pack(0, 0, 1),
        pack(0, 1, 1),
        pack(1, 2, 1),
        pack(1, 0, 1),
        pack(1, 1, 1),
    ]
}


#[allow(dead_code)] // Used only to generate the packed offset tables.
fn packi(x: i32, y: i32, z: i32) -> u64 {
    // Pack using the 2-complement.
    pack(x as u32, y as u32, z as u32)
}

pub struct SpGrid<T> {
    cell_width: f32,
    memory: MmapMut,
    _phantom: PhantomData<T>,
}

impl<T> SpGrid<T> {
    // For this first implementation, we use 4x4x4 blocks.
    // This will give us enough room to store more data into
    // the grid if we feel like it’s useful.
    // const BLOCK_SHAPE: [usize; DIM] = [4; DIM];

    const BLOCK_MX: u64 =
        0b0_001_001_001_001_001_001_001_001_001_001_001_001_001_001_001_001_001___000011_0000_00;
    const BLOCK_MY: u64 =
        0b0_010_010_010_010_010_010_010_010_010_010_010_010_010_010_010_010_010___001100_0000_00;
    const BLOCK_MZ: u64 =
        0b0_100_100_100_100_100_100_100_100_100_100_100_100_100_100_100_100_100___110000_0000_00;

    const PACK_HEADER: usize = 12;
    pub const PACK_ALIGN: usize = 6;
    pub const REGION_ID_MASK: u64 = !0b0___111111111111; // Mask that only retain the region part of the index.


    pub const PACKED_NEG_ONE: u64 = 18446744073709551552;
    pub const PACKED_NEG_TWO: u64 = 18446744073709550208;
    pub const PACKED_PLUS_FIVE: u64 = 30016;

    pub fn new(cell_width: f32) -> Result<Self> {
        info!(
            "Sizeof T: {}",
            std::mem::size_of::<T>() / std::mem::size_of::<u32>()
        );
        assert!(std::mem::size_of::<T>() <= 64);
 
        let memory = MmapMut::map_anon(usize::pow(2, 30))?;

        Ok(Self {
            memory,
            cell_width,
            _phantom: PhantomData,
        })
    }

    pub fn is_in_region_with_color(region_id: u64, color: u64) -> bool {
        let region_color_mask = 0b0111 << Self::PACK_HEADER;
        (region_id & region_color_mask) == (color << Self::PACK_HEADER)
    }

    pub fn is_index_valid(&self, index: usize) -> bool {
        index < self.memory.len()
    }

    pub fn cell_width(&self) -> f32 {
        self.cell_width
    }

    // pub fn nodes_mut(&mut self) -> impl Iterator<Item = &mut GridNode> {
    //     self.nodes.iter_mut()
    // }

    pub fn cell_pos_closest_to_point(&self, pt: &Vec3A) -> Vec3A {
        (*pt / self.cell_width).round() * self.cell_width
    }

    pub fn cell_at_point(&self, pt: &Vec3A) -> u64 {
        let coord = (*pt / self.cell_width).as_uvec3();
        return pack(coord.x, coord.y, coord.z);
    }

    pub fn cell_associated_to_point(&self, pt: &Vec3A) -> u64 {
        let coord = (*pt / self.cell_width).round().as_uvec3();
        return pack(
            coord.x.overflowing_sub(1).0,
            coord.y.overflowing_sub(1).0,
            coord.z.overflowing_sub(1).0,
        );
    }

    pub fn region_associated_to_point(&self, pt: &Vec3A) -> u64 {
        self.cell_associated_to_point(pt) & Self::REGION_ID_MASK
    }

    pub fn get(&self, cell: &UVec3) -> &T
    where
        T: Pod,
    {
        return self.get_packed(pack(cell.x, cell.y, cell.z));
    }

    pub fn get_mut(&mut self, cell: &UVec3) -> &mut T
    where
        T: Pod,
    {
        return self.get_packed_mut(pack(cell.x, cell.y, cell.z));
    }

    pub fn is_cell_index_valid(&self, cell_id: u64) -> bool {
        cell_id as usize + std::mem::size_of::<T>() <= self.memory.len()
    }

    pub fn cell_center(&self, cell_id: u64) -> Vec3A {
        Vec3A::from(unpack(cell_id).map(|e| e as f32)) * self.cell_width
    }

    pub fn get_packed(&self, cell_id: u64) -> &T
    where
        T: Pod,
    {
        let bytes = &self.memory[cell_id as usize..cell_id as usize + std::mem::size_of::<T>()];
        println!("{:?}", bytes);
        bytemuck::from_bytes(bytes)
    }

    pub fn get_packed_mut(&mut self, cell_id: u64) -> &mut T
    where
        T: Pod,
    {
        let bytes = &mut self.memory[cell_id as usize..cell_id as usize + std::mem::size_of::<T>()];
        bytemuck::from_bytes_mut(bytes)
    }

    pub fn is_neighborhood_valid(&self, cell_id: u64) -> bool {
        self.is_cell_index_valid(cell_id)
            && self.is_cell_index_valid(packed_add(cell_id, Self::PACKED_PLUS_FIVE))
    }

    pub fn shift_cell_neg_one(&self, cell_id: u64) -> u64 {
        packed_add(cell_id, Self::PACKED_NEG_ONE)
    }

    pub fn shift_cell_neg_two(&self, cell_id: u64) -> u64 {
        packed_add(cell_id, Self::PACKED_NEG_TWO)
    }

    pub fn region_neighbors(region_id: u64) -> [u64; 7] {
        [
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[0]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[1]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[2]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[3]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[4]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[5]),
            packed_add(region_id, PACKED_NBH_REGION_SHIFTS[6]),
        ]
    }

    #[inline(always)]
    pub fn for_each_neighbor_packed(&self, cell_id: u64, mut f: impl FnMut(u64, UVec3, &T))
    where
        T: Pod,
    {
        for (packed_shift, shift) in PACKED_NBH_SHIFTS.iter().zip(NBH_SHIFTS.iter()) {
            let adj_cell_id = packed_add(cell_id, *packed_shift);
            let grid_node = self.get_packed(adj_cell_id);
            f(adj_cell_id, *shift, grid_node)
        }
    }

    #[inline(always)]
    pub fn for_each_neighbor_packed_mut(
        &mut self,
        cell_id: u64,
        mut f: impl FnMut(u64, UVec3, &mut T),
    ) where
        T: Pod,
    {
        for (packed_shift, shift) in PACKED_NBH_SHIFTS.iter().zip(NBH_SHIFTS.iter()) {
            let adj_cell_id = packed_add(cell_id, *packed_shift);
            let grid_node = self.get_packed_mut(adj_cell_id);
            f(adj_cell_id, *shift, grid_node)
        }
    }

    pub fn memory(&mut self) -> &mut MmapMut {
        return &mut self.memory;
    }
}

fn masked_add<const MASK: u64>(i: u64, j: u64) -> u64 {
    (i & MASK).overflowing_add(j | !MASK).0 & MASK
}

fn packed_add(i: u64, j: u64) -> u64 {
    return masked_add::<{ SpGrid::<()>::BLOCK_MX }>(i, j)
        | masked_add::<{ SpGrid::<()>::BLOCK_MY }>(i, j)
        | masked_add::<{ SpGrid::<()>::BLOCK_MZ }>(i, j);
}

fn pack(x: u32, y: u32, z: u32) -> u64 {
    let mut res = crate::morton3::morton_encode3(x >> 2, y >> 2, z >> 2) as u64;

    res = res << SpGrid::<()>::PACK_HEADER;
    res = res | ((x as u64 & 0b0011) << SpGrid::<()>::PACK_ALIGN);
    res = res | ((y as u64 & 0b0011) << (SpGrid::<()>::PACK_ALIGN + 2));
    res | ((z as u64 & 0b0011) << (SpGrid::<()>::PACK_ALIGN + 4))
}

#[allow(dead_code)] // Useful for debugging.
fn unpack(xyz: u64) -> [u32; 3] {
    let [x, y, z] = crate::morton3::morton_decode3(xyz >> SpGrid::<()>::PACK_HEADER);

    [
        (x << 2) | ((xyz as u32 >> SpGrid::<()>::PACK_ALIGN) & 0b0011),
        (y << 2) | ((xyz as u32 >> (SpGrid::<()>::PACK_ALIGN + 2)) & 0b0011),
        (z << 2) | ((xyz as u32 >> (SpGrid::<()>::PACK_ALIGN + 4)) & 0b0011),
    ]
}

/*
 // See https://stackoverflow.com/a/58980803
fn interleave(a: u32, b: u32) -> u64 {
    (interleave(a) << 1) | space_out(b)
}

fn space_out(a: u32) -> u64 {
    let mut x = (a as u64) & 0x00000000FFFFFFFFL;
    x = (x | (x << 16)) & 0x0000FFFF0000FFFF;
    x = (x | (x <<  8)) & 0x00FF00FF00FF00FF;
    x = (x | (x <<  4)) & 0x0F0F0F0F0F0F0F0F;
    x = (x | (x <<  2)) & 0x3333333333333333;
    x = (x | (x <<  1)) & 0x5555555555555555;
    x
}
*/

#[cfg(test)]
mod test {
    use bevy::prelude::{info, UVec3};

    #[test]
    fn nbh_table_is_valid() {
        info!("{:?}", super::nbh_shifts());
        info!("{:?}", super::nbh_region_shifts());

        let packed_neg_one = super::packi(-1, -1, -1);
        info!("PACKED_NEG_ONE = {}", packed_neg_one);
        assert_eq!(packed_neg_one, super::SpGrid::<()>::PACKED_NEG_ONE);

        let packed_neg_two = super::packi(-2, -2, -2);
        info!("PACKED_NEG_TWO = {}", packed_neg_two);
        assert_eq!(packed_neg_two, super::SpGrid::<()>::PACKED_NEG_TWO);

        let packed_plus_five = super::pack(5, 5, 5);
        info!("PACKED_PLUS_FIVE = {}", packed_plus_five);
        assert_eq!(packed_plus_five, super::SpGrid::<()>::PACKED_PLUS_FIVE);

        // Check that our PACKED_NBH_SHIFTS table are in sync.
        // If not, that means we probably need that we changed our packing function, or changed
        // the table’s ordering, and forgot to call the nbh_shifts() to recompute these tables
        // and modify super::PACKED_NBH_SHIFTS accordingly.
        //
        // In that case, simply copy-paste the table printed above to the relevant 2D or
        // 3D PACKED_NBH_SHIFTS table.
        for (shift, pshift) in super::NBH_SHIFTS
            .iter()
            .zip(super::PACKED_NBH_SHIFTS)
        {
            assert_eq!(
                pshift,
                super::pack(shift.x as u32, shift.y as u32, shift.z as u32)
            );
        }

        for (shift, pshift) in super::NBH_REGION_SHIFTS
            .iter()
            .zip(super::PACKED_NBH_REGION_SHIFTS)
        {
            assert_eq!(
                pshift,
                super::pack(shift[0] as u32, shift[1] as u32, shift[2] as u32)
            );
        }
    }

    #[test]
    fn test_morton() {
        for i in 0..100 {
            for j in 0..50 {
                for k in 0..50 {
                    let coords = [i, j, k];
                    let packed = super::pack(coords[0], coords[1], coords[2]);
                    let unpacked = super::unpack(packed);
                    assert_eq!(coords, unpacked);
                }
            }
        }
    }

    #[test]
    fn test_morton_sum() {
        for k in 0..10 {
            let shft = 2_u32.pow(k);
            for i in 1 + shft..50 + shft {
                for j in 1 + shft..25 + shft {
                    for k in 1 + shft..25 + shft {
                        let coords = UVec3::from([i, j, k]);
                        let packed = super::pack(coords[0], coords[1], coords[2]);

                        for (shift, pshift) in super::NBH_SHIFTS
                            .iter()
                            .zip(super::PACKED_NBH_SHIFTS)
                        {
                            let psum = super::packed_add(packed, pshift);
                            let actual: [u32; 3] = (coords.as_ivec3() + shift.as_ivec3())
                                .to_array()
                                .map(|e| e as u32)
                                .into();
                            assert_eq!(actual, super::unpack(psum));
                        }
                    }
                }
            }
        }
    }
}