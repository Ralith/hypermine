use crate::node::{DualGraph, VoxelData};
use crate::{
    dodeca::{Side, Vertex},
    graph::NodeId,
    world::Material,
    Plane,
};

#[allow(dead_code)]

/*
The set of voxels that a collision body covers within a chunk
*/

pub struct ChunkBoundingBox {
    node: graph::NodeID, 
    chunk: dodeca::Vertex, 
    min_xyz na::Vector3<i32>,
    max_xyz na::Vector3<i32>,
 }


/*
The set of voxels that a collision body covers.
*/
pub struct BoundingBox {
    Vec<ChunkBoundingBox> boundingBoxes,
}


// Does not figure out which chunk it is in automatically
impl BoundingBox {
    pub fn createAABB(graph::NodeID startNode, dodeca::Vertex startChunk, Vector4<f64> position, f64 radius, graph: &DualGraph) -> Self {
        boundingBoxes = Vec<ChunkBoundingBox>::new();
        mut Option(ChunkBoundingBox) result;
        let sides[dodeca::Side; 3] = startChunk.canonical_sides();

        // get BBs for the chunks within the node.
        for v in Vertex {
            addSubBB(boundingBoxes,  getChunkBoundingBox( startNode, v, position, radius));
        }

        // get BBs for foreign chunks sharing a dodeca side
        for side in sides {
            let node = DualGraph.neighbor(startNode, side); // possible crash if ungenerated node is looked at.
            
            for v in side.vertices() {
                addSubBB(boundingBoxes, getChunkBoundingBox( node, v, position, radius));
            }
        }

        let oppositeNode = DualGraph.neighbor( DualGraph.neighbor( DualGraph.neighbor(startNode, sides[0]), sides[1]) );
        
        // get BBs for the chunks sharing an edge
        for side in sides {
            let node = DualGraph.neighbor(oppositeNode, side);

            addSubBB(boundingBoxes, getChunkBoundingBox( node, Vertex::from_sides(sides[0], sides[1], sides[2]), position, radius ));
        }

        // get BB for chunk sharing only a vertex.
        addSubBB(boundingBoxes, getChunkBoundingBox(oppositeNode, Vertex::from_sides(sides[0], sides[1], sides[2]), position, radius));

        self
    }

    // adds a sub-bounding box to a list if it exists
    pub fn addSubBB (list: &Vector3<ChunkBoundingBox>, result: Option(ChunkBoundingBox) ) {
        if result.is_some() list.push(result.unwrap());
    } 

}



// translatedPosition should be the object position in the node coordinates of the chunk.
// node can be easily factored out if it stops being convienent.
impl ChunkBoundingBox {
    pub fn getChunkBoundingBox (graph::NodeID node, dodeca::Vertex chunk, Vector4<f64> translatedPosition, f64 radius) -> Option<Self> {
        let na::Vector3<f64>. euclideanPosition = (chunk.chunk_to_node().try_inverse().unwrap() * translatedPosition).xyz;
        let mut na::Vector3<i32> min_xyz = na::Vector3::new();
        let mut na::Vector<i32> max_xyz = na::Vector3::new();
        let i32 dimension = 12; // should unhardcode this asap

        // verify at least one box corner is within the chunk
        if translatedPosition.iter().all(|n| n + radius > 0 && n - radius < 1) {
            min_xyz.x = (translatedPosition.x.min(0) * dimension ).floor() as i32;
            max_xyz.x = (translatedPosition.x.max(1) * dimension ).ceil() as i32;

            min_xyz.y = (translatedPosition.y.min(0) * dimension ).floor() as i32;
            max_xyz.y = (translatedPosition.y.max(1) * dimension ).ceil() as i32;

            min_xyz.z = (translatedPosition.z.min(0) * dimension ).floor() as i32;
            max_xyz.z = (translatedPosition.z.max(1) * dimension ).ceil() as i32;

            Some (
                ChunkBoundingBox { 
                    node: node,
                    chunk: chunk,
                    min_xyz: clone(min_xyz),
                    max_xyz: clone(max_xyz),
                }
            )
        }
        
        else None();
        
   }
}