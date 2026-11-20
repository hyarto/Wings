//! 程序化低多边形网格构建器（顶点色 + Flat Shading）。

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

#[derive(Default, Clone)]
pub struct MeshBuilder {
    pub pos: Vec<[f32; 3]>,
    pub nor: Vec<[f32; 3]>,
    pub col: Vec<[f32; 4]>,
    pub idx: Vec<u32>,
}

impl MeshBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// 带平面法线的三角形（逆时针为正面）。
    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, col: [f32; 4]) {
        let n = (b - a).cross(c - a).normalize_or_zero();
        let base = self.pos.len() as u32;
        for p in [a, b, c] {
            self.pos.push(p.to_array());
            self.nor.push(n.to_array());
            self.col.push(col);
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }

    pub fn tri_cols(&mut self, v: [Vec3; 3], cols: [[f32; 4]; 3]) {
        let n = (v[1] - v[0]).cross(v[2] - v[0]).normalize_or_zero();
        let base = self.pos.len() as u32;
        for i in 0..3 {
            self.pos.push(v[i].to_array());
            self.nor.push(n.to_array());
            self.col.push(cols[i]);
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }

    /// 四边形 a-b-c-d（逆时针）。
    pub fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, col: [f32; 4]) {
        self.tri(a, b, c, col);
        self.tri(a, c, d, col);
    }

    /// 任意旋转的长方体。
    pub fn cuboid(&mut self, center: Vec3, half: Vec3, rot: Quat, col: [f32; 4]) {
        let c = |sx: f32, sy: f32, sz: f32| center + rot * Vec3::new(half.x * sx, half.y * sy, half.z * sz);
        let (p000, p100, p110, p010) = (c(-1., -1., -1.), c(1., -1., -1.), c(1., 1., -1.), c(-1., 1., -1.));
        let (p001, p101, p111, p011) = (c(-1., -1., 1.), c(1., -1., 1.), c(1., 1., 1.), c(-1., 1., 1.));
        self.quad(p001, p101, p111, p011, col); // +z
        self.quad(p100, p000, p010, p110, col); // -z
        self.quad(p101, p100, p110, p111, col); // +x
        self.quad(p000, p001, p011, p010, col); // -x
        self.quad(p011, p111, p110, p010, col); // +y
        self.quad(p000, p100, p101, p001, col); // -y
    }

    pub fn aabb(&mut self, center: Vec3, half: Vec3, col: [f32; 4]) {
        self.cuboid(center, half, Quat::IDENTITY, col);
    }

    /// 低多边形椭球（八面体细分一次），用于云团。
    pub fn blob(&mut self, center: Vec3, radii: Vec3, col: [f32; 4]) {
        let dirs = [Vec3::X, Vec3::NEG_X, Vec3::Y, Vec3::NEG_Y, Vec3::Z, Vec3::NEG_Z];
        let faces = [(0, 2, 4), (4, 2, 1), (1, 2, 5), (5, 2, 0), (4, 3, 0), (1, 3, 4), (5, 3, 1), (0, 3, 5)];
        for (a, b, c) in faces {
            let v = |d: Vec3| center + d * radii;
            let (va, vb, vc) = (dirs[a], dirs[b], dirs[c]);
            // 每个面再细分为 4
            let (mab, mbc, mca) = ((va + vb).normalize(), (vb + vc).normalize(), (vc + va).normalize());
            self.tri(v(va), v(mab), v(mca), col);
            self.tri(v(mab), v(vb), v(mbc), col);
            self.tri(v(mca), v(mbc), v(vc), col);
            self.tri(v(mab), v(mbc), v(mca), col);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.idx.is_empty()
    }

    pub fn build(self) -> Mesh {
        Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.pos)
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.nor)
            .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, self.col)
            .with_inserted_indices(Indices::U32(self.idx))
    }
}

pub fn rgba(c: [f32; 3]) -> [f32; 4] {
    [c[0], c[1], c[2], 1.0]
}
