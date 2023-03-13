//! Postcard doesn't support serializing to an existing vec out of the box

use postcard::Result;

pub fn serialize<T: serde::Serialize + ?Sized>(value: &T, vec: &mut Vec<u8>) -> Result<()> {
    postcard::serialize_with_flavor(value, ExtendVec(vec))
}

struct ExtendVec<'a>(&'a mut Vec<u8>);

impl postcard::ser_flavors::Flavor for ExtendVec<'_> {
    type Output = ();

    fn try_push(&mut self, data: u8) -> Result<()> {
        self.0.push(data);
        Ok(())
    }

    fn finalize(self) -> Result<()> {
        Ok(())
    }

    fn try_extend(&mut self, data: &[u8]) -> Result<()> {
        self.0.extend_from_slice(data);
        Ok(())
    }
}
