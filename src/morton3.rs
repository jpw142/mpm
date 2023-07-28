// Didn't wanna mess anything up, straight copy and paste
// https://github.com/dimforge/sparkl/blob/4517c9f96acf2e7588b31dbab744a616608af2af/src/utils/morton3.rs

// This spreads out a binary number with 2 0's inbetween each of its numbers
// abcd becomes
// 0a0b0c0d
fn spread(mut w: u64) -> u64 {
    w &= 0x00000000001fffff;
    w = (w | w << 32) & 0x001f00000000ffff;
    w = (w | w << 16) & 0x001f0000ff0000ff;
    w = (w | w << 8) & 0x010f00f00f00f00f;
    w = (w | w << 4) & 0x10c30c30c30c30c3;
    w = (w | w << 2) & 0x1249249249249249;
    w
}

// This spreads the indexes and then interleaves them 
// X00X00X
// 0Y00Y00Y
// 00Z00Z00Z
// XYZXYZXYZ
pub fn morton_encode3(x: u32, y: u32, z: u32) -> u64 {
    spread(x as u64) | (spread(y as u64) << 1) | (spread(z as u64) << 2)
}

// Decoding

fn compact(mut w: u64) -> u32 {
    w &= 0x1249249249249249;
    w = (w ^ (w >> 2)) & 0x30c30c30c30c30c3;
    w = (w ^ (w >> 4)) & 0xf00f00f00f00f00f;
    w = (w ^ (w >> 8)) & 0x00ff0000ff0000ff;
    w = (w ^ (w >> 16)) & 0x00ff00000000ffff;
    w = (w ^ (w >> 32)) & 0x00000000001fffff;
    w as u32
}

pub fn morton_decode3(code: u64) -> [u32; 3] {
    [compact(code), compact(code >> 1), compact(code >> 2)]
}