const NEIGHBORHOOD: [[bool; 5]; 5] = [
    [false, true, false, false, true],
    [true, false, true, false, false],
    [false, true, false, true, false],
    [false, false, true, false, true],
    [true, false, false, true, false],
];

#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Side {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
}

fn adjacent(a: Side, b: Side) -> bool {
    unsafe {
        *NEIGHBORHOOD
            .get_unchecked(a as usize)
            .get_unchecked(b as usize)
    }
}

#[derive(Debug, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct NodeString {
    path: Vec<Side>,
}

impl NodeString {
    pub fn new() -> Self {
        Self { path: vec![] }
    }

    pub fn append(&mut self, new_segment: Side) {
        let mut insertion_point = self.path.len();

        for (index, &segment) in self.path.iter().enumerate().rev() {
            if segment == new_segment {
                self.path.remove(index);
                return;
            }

            if !adjacent(segment, new_segment) {
                break;
            }

            if new_segment < segment {
                insertion_point = index;
            }
        }

        self.path.insert(insertion_point, new_segment);
    }

    pub fn backtrack(&mut self) {
        self.path.pop();
    }

    pub fn has_child(&self, new_segment: Side) -> bool {
        for &segment in self.path.iter().rev() {
            if segment == new_segment {
                return false;
            }

            if !adjacent(segment, new_segment) {
                return true;
            }

            if new_segment < segment {
                return false;
            }
        }
        true
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }
}

impl Default for NodeString {
    fn default() -> Self {
        NodeString::new()
    }
}
