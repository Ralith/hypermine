use libm::{cosf, sinf, sqrtf};
use rand::{Rng, SeedableRng};
use rand_distr::Poisson;
use rand_pcg::Pcg64Mcg;

use crate::{
    dodeca::{Side, Vertex},
    graph::{Graph, NodeId},
    math::{MDirection, MIsometry, MPoint, MVector},
    node::VoxelData,
    peer_traverser,
    voxel_math::Coords,
    world::Material,
    worldgen::hash,
};

/// Whether an assortment of random horospheres should be added to world generation. This is a temporary
/// option until large structures that fit with the theme of the world are introduced.
/// For code simplicity, this is made into a constant instead of a configuration option.
const HOROSPHERES_ENABLED: bool = false;

/// Value to mix into the node's spice for generating horospheres. Chosen randomly.
const HOROSPHERE_SEED: u64 = 6046133366614030452;

/// Represents a node's reference to a particular horosphere. As a general rule, for any give horosphere,
/// every node in the convex hull of nodes containing the horosphere will have a `HorosphereNode`
/// referencing it. The unique node in this convex hull with the smallest depth in the graph is the owner
/// of the horosphere, where it is originally generated.
#[derive(Copy, Clone)]
pub struct HorosphereNode {
    /// The node that originally created the horosphere. All parts of the horosphere will
    /// be in a node with this as an ancestor, and all HorosphereNodes with the same `owner` correspond
    /// to the same horosphere.
    owner: NodeId,

    /// The horosphere's location relative to the node containing this `HorosphereNode`
    horosphere: Horosphere,

    /// A region that bounds the `HorosphereNode`'s descendents. A `HorosphereNode` will never propagate beyond
    /// this region, and the region's bounds will be as tight as possible. Note that this region does note necessarily
    /// contain the whole horosphere because parts of the horosphere that require backtracking towards the origin
    /// are ignored.
    // Note: All public constructors generate a `HorosphereNode` with tight bounds, but some `HorosphereNode`s
    // might not have tight bounds because a `HorosphereNode` is used in an intermediate calculations, averaged
    // together with other `HorosphereNode`s before the bounds are tightened.
    region: NodeBoundedRegion,
}

impl HorosphereNode {
    /// Returns the `HorosphereNode` for the given node, either by propagating an existing parent
    /// `HorosphereNode` or by randomly generating a new one.
    pub fn new(graph: &Graph, node_id: NodeId) -> Option<HorosphereNode> {
        if !HOROSPHERES_ENABLED {
            return None;
        }
        HorosphereNode::create_from_parents(graph, node_id)
            .or_else(|| HorosphereNode::maybe_create_fresh(graph, node_id))
    }

    /// Propagates `HorosphereNode` information from the given parent nodes to this child node. Returns
    /// `None` if there's no horosphere to propagate, either because none of the parent nodes have a
    /// horosphere associated with them, or because any existing horosphere is outside the range
    /// of this node.
    fn create_from_parents(graph: &Graph, node_id: NodeId) -> Option<HorosphereNode> {
        // Rather than selecting an arbitrary parent HorosphereNode, we average all of them. This
        // is important because otherwise, the propagation of floating point precision errors could
        // create a seam. This ensures that all errors average out, keeping the horosphere smooth.
        let mut horospheres_to_average_iter =
            graph
                .parents(node_id)
                .filter_map(|(parent_side, parent_id)| {
                    graph
                        .node_state(parent_id)
                        .horosphere
                        .as_ref()
                        .and_then(|h| h.propagate(parent_side))
                });

        let mut horosphere_node = horospheres_to_average_iter.next()?;
        let mut count = 1;
        for other in horospheres_to_average_iter {
            // Take an average of all HorosphereNodes in this iterator, giving each of them equal weight
            // by keeping track of a moving average with a weight that changes over time to make the
            // numbers work out the same way.
            count += 1;
            horosphere_node.average_with(other, 1.0 / count as f32);
        }

        horosphere_node.horosphere.renormalize();
        horosphere_node.tighten_region_bounds();
        Some(horosphere_node)
    }

    /// Create a `HorosphereNode` corresponding to a freshly created horosphere with the given node as its owner,
    /// if one should be created. This function is called on every node that doesn't already have a horosphere
    /// associated with it, so this function has control over how frequent the horospheres should be.
    fn maybe_create_fresh(graph: &Graph, node_id: NodeId) -> Option<HorosphereNode> {
        const HOROSPHERE_DENSITY: f32 = 6.0;

        let spice = graph.hash_of(node_id) as u64;
        let mut rng = rand_pcg::Pcg64Mcg::seed_from_u64(hash(spice, HOROSPHERE_SEED));
        for _ in 0..rng.sample(Poisson::new(HOROSPHERE_DENSITY).unwrap()) as u32 {
            // This logic is designed to create an average of "HOROSPHERE_DENSITY" horosphere candiates
            // in the region determined by `random_horosphere_pos` and then filters the resulting
            // list of candiates to only ones where the current node is the suitable owner for them.
            // Filtering instead of rejection sampling ensures a uniform distribution of horosphere
            // even though different nodes have different-sized regions for valid horospheres.

            // However, we do return early to ensure that after filtering, we only take the first
            // horosphere if there is one, since a node can have at most one horosphere.
            let horosphere = Horosphere::new_random(&mut rng, MAX_OWNED_HOROSPHERE_W);
            if is_horosphere_valid(graph, node_id, &horosphere) {
                let mut horosphere_node = HorosphereNode {
                    owner: node_id,
                    horosphere,
                    region: NodeBoundedRegion::node_and_descendents(graph, node_id),
                };
                horosphere_node.tighten_region_bounds();
                return Some(horosphere_node);
            }
        }
        None
    }

    /// Updates the region associated with the `HorosphereNode` to have bounds that are as tight as possible.
    fn tighten_region_bounds(&mut self) {
        for side in Side::iter() {
            if !self.region.is_bounded_by(side) && self.can_tighten_region_bound(side) {
                self.region.add_bound(side);
            }
        }
    }

    /// Computes whether propagation can stop at a particular side due to no part of the horosphere
    /// being behind it. This function is used to tighten region bounds.
    fn can_tighten_region_bound(&self, side: Side) -> bool {
        !self.horosphere.intersects_half_space(side.normal())
    }

    /// Returns an estimate of the `HorosphereNode` corresponding to the node adjacent to the current node
    /// at the given side, or `None` if the horosphere is no longer relevant after crossing the given side.
    /// The estimates given by multiple nodes may be used to produce the actual `HorosphereNode`.
    fn propagate(&self, side: Side) -> Option<HorosphereNode> {
        // Don't propagate beyond the already-computed bounds of the `HorosphereNode`.
        if self.region.is_bounded_by(side) {
            return None;
        }

        Some(HorosphereNode {
            owner: self.owner,
            horosphere: side.reflection() * self.horosphere,
            region: self.region.neighbor(side),
        })
    }

    /// Takes the weighted average of the coordinates of this horosphere with the coordinates of the other horosphere.
    fn average_with(&mut self, other: HorosphereNode, other_weight: f32) {
        if self.owner != other.owner {
            // If this panic is triggered, it may mean that two horospheres were generated that interfere
            // with each other. The logic in `should_generate` should prevent this, so this would be a sign
            // of a bug in that function's implementation.
            panic!("Tried to average two unrelated HorosphereNodes");
        }
        self.horosphere.pos =
            self.horosphere.pos * (1.0 - other_weight) + other.horosphere.pos * other_weight;
        self.region = self.region.intersect(other.region);
    }

    /// Returns whether the horosphere is freshly created, instead of a
    /// reference to a horosphere created earlier on in the node graph.
    fn is_fresh(&self, node_id: NodeId) -> bool {
        self.owner == node_id
    }

    /// If `self` and `other` would propagate to the same node, to avoid interference, only one of these
    /// two horospheres can generate. This function determines whether `self` should be the one to generate.
    fn has_priority(&self, other: &HorosphereNode, node_id: NodeId) -> bool {
        // If both horospheres are fresh, use the owner's NodeId as an arbitrary
        // tie-breaker to decide which horosphere should win.
        !self.is_fresh(node_id) || (other.is_fresh(node_id) && self.owner < other.owner)
    }

    /// Based on other nodes in the graph, determines whether the horosphere
    /// should generate. If false, it means that another horosphere elsewhere
    /// would interfere, and generation should not proceed.
    pub fn should_generate(&self, graph: &Graph, node_id: NodeId) -> bool {
        if !self.is_fresh(node_id) {
            // The horosphere is propagated and so is already proven to exist.
            return true;
        }

        for peer in peer_traverser::expect_peer_nodes(graph, node_id) {
            let Some(peer_horosphere) = graph
                .partial_node_state(peer.node())
                .candidate_horosphere
                .as_ref()
            else {
                continue;
            };
            if !self.has_priority(peer_horosphere, node_id)
                // Check that these horospheres can interfere by seeing if their regions share a node in common.
                && peer_horosphere.region.contains_node(peer.peer_to_shared())
                && self.region.contains_node(peer.base_to_shared())
            {
                return false;
            }
        }
        true
    }
}

/// Returns whether the given horosphere position could represent a horosphere generated by the
/// given node. The requirement is that a horosphere must be bounded by all of the node's parent sides
/// (as otherwise, a parent node would own the horosphere), and the horosphere must not be fully
/// behind any of the other dodeca sides (as otherwise, a child node would own the horosphere). Note
/// that the horosphere does not necessarily need to intersect the dodeca to be valid.
fn is_horosphere_valid(graph: &Graph, node_id: NodeId, horosphere: &Horosphere) -> bool {
    Side::iter().all(|s| !horosphere.is_inside_half_space(s.normal()))
        && (graph.parents(node_id)).all(|(s, _)| !horosphere.intersects_half_space(s.normal()))
}

/// The maximum node-centric w-coordinate a horosphere can have such that the node in question
/// is still the owner of the horosphere.
// See `test_max_owned_horosphere_w()` for how this is computed.
const MAX_OWNED_HOROSPHERE_W: f32 = 5.9047837;

/// Represents a chunks's reference to a particular horosphere.
pub struct HorosphereChunk {
    /// The horosphere's location relative to the chunk containing this `HorosphereChunk`.
    pub horosphere: Horosphere,
}

impl HorosphereChunk {
    /// Creates a `HorosphereChunk` based on a `HorosphereNode`
    pub fn new(horosphere_node: &HorosphereNode, vertex: Vertex) -> Self {
        HorosphereChunk {
            horosphere: vertex.node_to_dual() * horosphere_node.horosphere,
        }
    }

    /// Rasterizes the horosphere chunk into the given `VoxelData`
    pub fn generate(&self, voxels: &mut VoxelData, chunk_size: u8) {
        for z in 0..chunk_size {
            for y in 0..chunk_size {
                for x in 0..chunk_size {
                    let pos = MVector::new(
                        x as f32 + 0.5,
                        y as f32 + 0.5,
                        z as f32 + 0.5,
                        chunk_size as f32 * Vertex::dual_to_chunk_factor(),
                    )
                    .normalized_point();
                    if self.horosphere.contains_point(&pos) {
                        voxels.data_mut(chunk_size)[Coords([x, y, z]).to_index(chunk_size)] =
                            Material::RedSandstone;
                    }
                }
            }
        }
    }
}

/// A horosphere in hyperbolic space. Contains helper functions for common operations
#[derive(Copy, Clone)]
pub struct Horosphere {
    /// This vector, `pos`, entirely represents the horosphere using the following rule: A vector
    /// `point` is in this horosphere exactly when `point.mip(&self.pos) == -1`. This vector should
    /// always have the invariant `self.pos.mip(&self.pos) == 0`, behaving much like a "light-like"
    /// vector in Minkowski space.
    ///
    /// One recommended way to gain an intuition of this vector is to consider its direction separately.
    /// The vector points in the direction of an ideal point in the hyperboloid model because it is on
    /// the cone that the hyperboloid approaches. This ideal point is the center of the horosphere.
    /// This determines `self.pos` up to a scalar multiple, and the remaining degree of freedom can
    /// be pinned down by analyzing the w-coordinate.
    ///
    /// The w-coordinate determines which of the concentric horospheres with that ideal point is represented.
    /// A larger w-coordinate represents a horosphere that is farther away from the origin.
    /// If the w-coordinate is 1, the origin is on the horosphere's surface.
    /// If it's less than 1, the origin is in the horosphere's interior, and if it's greater than 1, the origin
    /// is outside the horosphere.
    ///
    /// TODO: If a player traverses too far inside a horosphere, this vector will underflow, preventing
    /// the horosphere from generating properly. Fixing this requires using logic similar to `Plane` to
    /// increase the range of magnitudes the vector can take.
    pos: MVector<f32>,
}

impl Horosphere {
    /// Returns whether the point is inside the horosphere
    pub fn contains_point(&self, point: &MPoint<f32>) -> bool {
        self.pos.mip(point) >= -1.0
    }

    /// Returns whether the horosphere is entirely inside the half space in front of the plane defined by `normal`
    pub fn is_inside_half_space(&self, normal: &MDirection<f32>) -> bool {
        self.pos.mip(normal) >= 1.0
    }

    /// Returns whether any part of the horosphere intersects the half space in front of the plane defined by `normal`
    pub fn intersects_half_space(&self, normal: &MDirection<f32>) -> bool {
        self.pos.mip(normal) >= -1.0
    }

    /// Ensures that the horosphere invariant holds (`pos.mip(&pos) == 0`), as numerical error can otherwise propagate,
    /// potentially making the surface behave more like a sphere or an equidistant surface.
    pub fn renormalize(&mut self) {
        self.pos.w = self.pos.xyz().norm();
    }

    /// Returns a uniformly random horosphere, restricted so that the w-coordinate of its representing vector
    /// must be at most `max_w`.
    pub fn new_random(rng: &mut Pcg64Mcg, max_w: f32) -> Self {
        // Pick a w-coordinate whose probability density function is `p(w) = w`. By trial and error,
        // this seems to produce horospheres with a uniform and isotropic distribution.
        // TODO: Find a rigorous explanation for this. We would want to show that the probability density is unchanged
        // when an isometry is applied.
        let w = sqrtf(rng.random::<f32>()) * max_w;

        // Uniformly pick spherical coordinates from a unit sphere
        let cos_phi = rng.random::<f32>() * 2.0 - 1.0;
        let sin_phi = sqrtf(1.0 - cos_phi * cos_phi);
        let theta = rng.random::<f32>() * std::f32::consts::TAU;

        // Construct the resulting vector.
        Horosphere {
            pos: MVector::new(
                w * sin_phi * cosf(theta),
                w * sin_phi * sinf(theta),
                w * cos_phi,
                w,
            ),
        }
    }
}

impl std::ops::Mul<Horosphere> for &MIsometry<f32> {
    type Output = Horosphere;

    fn mul(self, rhs: Horosphere) -> Self::Output {
        Horosphere {
            pos: self * rhs.pos,
        }
    }
}

/// Represents a region of space bounded by planes corresponding to a subset of a node's sides in that node's perspective.
#[derive(Clone, Copy)]
struct NodeBoundedRegion {
    /// A bit-array with 12 elements, one for each side. A 1 means that that side is a bound, and a 0 means it is not.
    bounded_sides: u16,
}

impl NodeBoundedRegion {
    /// Creates a region with no bounds
    #[cfg(test)]
    fn unbounded() -> Self {
        NodeBoundedRegion { bounded_sides: 0 }
    }

    /// Creates a region that contains the given node and all its descendents
    fn node_and_descendents(graph: &Graph, node_id: NodeId) -> Self {
        let mut bounded_sides = 0;
        for (parent_side, _) in graph.parents(node_id) {
            bounded_sides |= 1 << (parent_side as u8);
        }
        NodeBoundedRegion { bounded_sides }
    }

    /// Produces the set intersection of the `self` and `other` regions
    fn intersect(self, other: NodeBoundedRegion) -> NodeBoundedRegion {
        NodeBoundedRegion {
            bounded_sides: self.bounded_sides | other.bounded_sides,
        }
    }

    /// Returns the sub-region consisting of everything beyond the plane containing
    /// the given side (in the perspective of the corresponding neighboring node).
    /// As a precondition, the given side cannot be an existing bound, as that would
    /// make the sub-region empty (which is non-representable in `NodeBoundedRegion`).
    fn neighbor(self, neighbor_side: Side) -> NodeBoundedRegion {
        debug_assert!(!self.is_bounded_by(neighbor_side));

        let mut bounded_sides = self.bounded_sides;

        // Don't allow backtracking
        bounded_sides |= 1 << (neighbor_side as u8);

        // As we're shifting perspective to a neighboring node, most sides now refer
        // to different planes and no longer bound the region. The only exceptions to this
        // are the shared side and its neighbors.
        for side in Side::iter() {
            if !side.adjacent_to(neighbor_side) && side != neighbor_side {
                bounded_sides &= !(1 << (side as u8));
            }
        }
        NodeBoundedRegion { bounded_sides }
    }

    /// Returns whether the node reachable via the given path is within the region.
    /// Note that this path is required to be one of the shortest paths that can reach
    /// that node.
    fn contains_node(self, path: impl Iterator<Item = Side>) -> bool {
        let mut current_region = self;
        for side in path {
            if current_region.is_bounded_by(side) {
                return false;
            }
            current_region = current_region.neighbor(side);
        }
        true
    }

    /// Returns whether the given side bounds the region
    fn is_bounded_by(self, side: Side) -> bool {
        self.bounded_sides & (1 << (side as u8)) != 0
    }

    /// Adds the given side as a bound for the region
    fn add_bound(&mut self, side: Side) {
        self.bounded_sides |= 1 << (side as u8);
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::math::MPoint;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_max_owned_horosphere_w() {
        // This tests that `MAX_OWNED_HOROSPHERE_W` is set to the correct value.

        // The worst case scenario would be a horosphere located directly in the direction of a dodeca's vertex.
        // This is because the horosphere can be outside the dodeca, tangent to each of the planes that extend the
        // dodeca's sides adjancent to that vertex. If that horosphere were brought any closer, it would intersect
        // all three of those planes, making it impossible for any child node to own the dodeca and forcing the node
        // in focus to own it.

        // First, find an arbitrary horosphere in the direction of a vertex.
        let example_vertex = Vertex::A;
        let example_vertex_pos = example_vertex.dual_to_node() * MPoint::origin();
        let mut horosphere_pos = MVector::from(example_vertex_pos);
        horosphere_pos.w = horosphere_pos.xyz().norm();

        // Then, scale the horosphere so that it's mip with each of the sides of the vertex is 1, making it tangent.
        horosphere_pos /= horosphere_pos.mip(example_vertex.canonical_sides()[0].normal());
        for side in example_vertex.canonical_sides() {
            assert_abs_diff_eq!(horosphere_pos.mip(side.normal()), 1.0, epsilon = 1.0e-6);
        }

        // Finally, compare that horosphere's w-coordinate to `MAX_OWNED_HOROSPHERE_W`
        assert_abs_diff_eq!(horosphere_pos.w, MAX_OWNED_HOROSPHERE_W, epsilon = 1.0e-6);
    }

    #[test]
    fn node_bounded_region_intersect_example() {
        let mut region0 = NodeBoundedRegion::unbounded();
        region0.add_bound(Side::A);
        region0.add_bound(Side::B);

        let mut region1 = NodeBoundedRegion::unbounded();
        region1.add_bound(Side::B);
        region1.add_bound(Side::C);

        let intersection = region0.intersect(region1);
        for side in Side::iter() {
            assert_eq!(
                intersection.is_bounded_by(side),
                [Side::A, Side::B, Side::C].contains(&side),
                "testing side {side:?}",
            );
        }
    }

    #[test]
    fn node_bounded_region_neighbor_example() {
        let mut region = NodeBoundedRegion::unbounded();
        // Sides adjacent to A
        region.add_bound(Side::B);
        region.add_bound(Side::C);
        region.add_bound(Side::D);

        // Sides not adjacent to A
        region.add_bound(Side::F);
        region.add_bound(Side::G);
        region.add_bound(Side::J);

        let neighbor = region.neighbor(Side::A);

        for side in Side::iter() {
            assert_eq!(
                neighbor.is_bounded_by(side),
                [Side::A, Side::B, Side::C, Side::D].contains(&side),
                "testing side {side:?}",
            );
        }
    }

    #[test]
    fn node_bounded_region_node_and_descendents_example() {
        let mut graph = Graph::new(1);
        let node_id = graph.ensure_neighbor(NodeId::ROOT, Side::A);
        let region = NodeBoundedRegion::node_and_descendents(&graph, node_id);

        for side in Side::iter() {
            assert_eq!(
                region.is_bounded_by(side),
                [Side::A].contains(&side),
                "testing side {side:?}",
            );
        }
    }

    #[test]
    fn node_bounded_region_contains_node_example() {
        let mut region = NodeBoundedRegion::unbounded();
        // Sides adjacent to A
        region.add_bound(Side::B);
        region.add_bound(Side::C);

        // Sides not adjacent to A
        region.add_bound(Side::F);
        region.add_bound(Side::G);
        region.add_bound(Side::J);

        assert!(region.contains_node([Side::A].into_iter()));
        assert!(!region.contains_node([Side::B].into_iter()));
        assert!(region.contains_node([Side::A, Side::F].into_iter()));
        assert!(!region.contains_node([Side::A, Side::B].into_iter()));
    }
}
