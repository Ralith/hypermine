#[allow(clippy::derive_partial_eq_without_eq)]
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Meta {
    /// Number of voxels along the edge of a chunk
    #[prost(uint32, tag = "1")]
    pub chunk_size: u32,
}
#[allow(clippy::derive_partial_eq_without_eq)]
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Node {
    /// Entities whose origins lie within this node
    #[prost(message, repeated, tag = "1")]
    pub archetypes: ::prost::alloc::vec::Vec<Archetype>,
    /// Voxel data for each modified chunk
    #[prost(message, repeated, tag = "2")]
    pub chunks: ::prost::alloc::vec::Vec<Chunk>,
}
/// A set of entities, all of which have the same components
#[allow(clippy::derive_partial_eq_without_eq)]
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Archetype {
    /// Entity IDs
    #[prost(fixed64, repeated, tag = "1")]
    pub entities: ::prost::alloc::vec::Vec<u64>,
    /// Type of components stored in each column
    #[prost(enumeration = "ComponentType", repeated, tag = "2")]
    pub component_types: ::prost::alloc::vec::Vec<i32>,
    /// Each data represents a dense column of component values of the type identified by the
    /// component_type at the same index as the column
    #[prost(bytes = "vec", repeated, tag = "3")]
    pub component_data: ::prost::alloc::vec::Vec<::prost::alloc::vec::Vec<u8>>,
}
#[allow(clippy::derive_partial_eq_without_eq)]
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct Chunk {
    /// Which dodecahedron vertex is associated with this chunk
    #[prost(uint32, tag = "1")]
    pub vertex: u32,
    /// Dense 3D array of 16-bit material tags for all voxels in this chunk
    #[prost(bytes = "vec", tag = "2")]
    pub voxels: ::prost::alloc::vec::Vec<u8>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, ::prost::Enumeration)]
#[repr(i32)]
pub enum ComponentType {
    Position = 0,
}
impl ComponentType {
    /// String value of the enum field names used in the ProtoBuf definition.
    ///
    /// The values are not transformed in any way and thus are considered stable
    /// (if the ProtoBuf definition does not change) and safe for programmatic use.
    pub fn as_str_name(&self) -> &'static str {
        match self {
            ComponentType::Position => "POSITION",
        }
    }
    /// Creates an enum from field names used in the ProtoBuf definition.
    pub fn from_str_name(value: &str) -> ::core::option::Option<Self> {
        match value {
            "POSITION" => Some(Self::Position),
            _ => None,
        }
    }
}
