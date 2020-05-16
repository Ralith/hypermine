/// A random-access table that maintains an LRU list in constant time
pub struct LruSlab<T> {
    slots: Box<[Slot<T>]>,
    /// Most recently used
    head: SlotId,
    /// Least recently used
    tail: SlotId,
    /// First unused
    free: SlotId,
    /// Number of occupied slots
    len: u32,
}

impl<T> LruSlab<T> {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(capacity: u32) -> Self {
        assert!(capacity != u32::max_value(), "capacity too large");
        Self {
            slots: (0..capacity)
                .map(|n| Slot {
                    value: None,
                    prev: SlotId::NONE,
                    next: if n + 1 == capacity {
                        SlotId::NONE
                    } else {
                        SlotId(n + 1)
                    },
                })
                .collect::<Vec<_>>()
                .into(),
            head: SlotId::NONE,
            tail: SlotId::NONE,
            free: if capacity == 0 {
                SlotId::NONE
            } else {
                SlotId(0)
            },
            len: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn len(&self) -> u32 {
        self.len
    }

    pub fn capacity(&self) -> u32 {
        self.slots.len() as u32
    }

    /// Inserts a value, returning the slot it was stored in
    ///
    /// The returned slot is marked as the most recently used.
    pub fn insert(&mut self, value: T) -> SlotId {
        let id = match self.alloc() {
            Some(id) => id,
            None => {
                let len = self.capacity();
                let cap = 2 * len.max(2);
                self.slots = self
                    .slots
                    .iter_mut()
                    .map(|x| Slot {
                        value: x.value.take(),
                        next: x.next,
                        prev: x.prev,
                    })
                    .chain((len..cap).map(|n| Slot {
                        value: None,
                        prev: SlotId::NONE,
                        next: if n + 1 == cap {
                            SlotId::NONE
                        } else {
                            SlotId(n + 1)
                        },
                    }))
                    .collect::<Vec<_>>()
                    .into_boxed_slice();
                self.free = SlotId(len + 1);
                SlotId(len)
            }
        };
        let idx = id.0 as usize;

        debug_assert!(self.slots[idx].value.is_none(), "corrupt free list");
        self.slots[idx].value = Some(value);
        self.link_at_head(id);
        self.len += 1;

        id
    }

    /// Get the least recently used slot, if any
    pub fn lru(&self) -> Option<SlotId> {
        if self.tail == SlotId::NONE {
            debug_assert_eq!(self.head, SlotId::NONE);
            None
        } else {
            Some(self.tail)
        }
    }

    pub fn remove(&mut self, slot: SlotId) -> T {
        self.unlink(slot);
        self.slots[slot.0 as usize].next = self.free;
        self.slots[slot.0 as usize].prev = SlotId::NONE;
        self.free = slot;
        self.len -= 1;
        self.slots[slot.0 as usize]
            .value
            .take()
            .expect("removing empty slot")
    }

    /// Mark `slot` as the most recently used and access it uniquely
    pub fn get_mut(&mut self, slot: SlotId) -> &mut T {
        self.freshen(slot);
        self.peek_mut(slot)
    }

    /// Access `slot` without marking it as most recently used
    pub fn peek(&self, slot: SlotId) -> &T {
        self.slots[slot.0 as usize].value.as_ref().unwrap()
    }

    /// Access `slot` uniquely without marking it as most recently used
    pub fn peek_mut(&mut self, slot: SlotId) -> &mut T {
        self.slots[slot.0 as usize].value.as_mut().unwrap()
    }

    /// Walks the container from most to least recently used
    pub fn iter(&self) -> Iter<'_, T> {
        Iter {
            slots: &self.slots[..],
            head: self.head,
            tail: self.tail,
            len: self.len,
        }
    }

    /// Remove a slot from the freelist
    fn alloc(&mut self) -> Option<SlotId> {
        if self.free == SlotId::NONE {
            return None;
        }
        let slot = self.free;
        self.free = self.slots[slot.0 as usize].next;
        Some(slot)
    }

    /// Mark `slot` as the most recently used
    fn freshen(&mut self, slot: SlotId) {
        if self.slots[slot.0 as usize].prev == SlotId::NONE {
            // This is already the freshest slot, so we don't need to do anything
            debug_assert_eq!(self.head, slot, "corrupt lru list");
            return;
        }

        self.unlink(slot);
        self.link_at_head(slot);
    }

    /// Add a link to the head of the list
    fn link_at_head(&mut self, slot: SlotId) {
        let idx = slot.0 as usize;
        if self.head == SlotId::NONE {
            // List was empty
            self.slots[idx].next = SlotId::NONE;
            self.tail = slot;
        } else {
            self.slots[idx].next = self.head;
            self.slots[self.head.0 as usize].prev = slot;
        }
        self.slots[idx].prev = SlotId::NONE;
        self.head = slot;
    }

    /// Remove a link from anywhere in the list
    fn unlink(&mut self, slot: SlotId) {
        let idx = slot.0 as usize;
        if self.slots[idx].prev != SlotId::NONE {
            self.slots[self.slots[idx].prev.0 as usize].next = self.slots[idx].next;
        } else {
            self.head = self.slots[idx].next;
        }
        if self.slots[idx].next != SlotId::NONE {
            self.slots[self.slots[idx].next.0 as usize].prev = self.slots[idx].prev;
        } else {
            // This was the tail
            self.tail = self.slots[idx].prev;
        }
    }
}

impl<T> Default for LruSlab<T> {
    fn default() -> Self {
        Self::new()
    }
}

struct Slot<T> {
    value: Option<T>,
    /// Next slot in the LRU or free list
    next: SlotId,
    /// Previous slot in the LRU list; NONE when free
    prev: SlotId,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SlotId(pub u32);

impl SlotId {
    const NONE: Self = SlotId(u32::max_value());
}

pub struct Iter<'a, T> {
    slots: &'a [Slot<T>],
    head: SlotId,
    tail: SlotId,
    len: u32,
}

impl<'a, T> Iterator for Iter<'a, T> {
    type Item = &'a T;
    fn next(&mut self) -> Option<&'a T> {
        if self.len == 0 {
            return None;
        }
        let idx = self.head.0 as usize;
        let result = self.slots[idx].value.as_ref().expect("corrupt LRU list");
        self.head = self.slots[idx].next;
        self.len -= 1;
        Some(result)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len as usize, Some(self.len as usize))
    }
}

impl<'a, T> DoubleEndedIterator for Iter<'a, T> {
    fn next_back(&mut self) -> Option<&'a T> {
        if self.len == 0 {
            return None;
        }
        let idx = self.tail.0 as usize;
        let result = self.slots[idx].value.as_ref().expect("corrupt LRU list");
        self.tail = self.slots[idx].prev;
        self.len -= 1;
        Some(result)
    }
}

impl<T> ExactSizeIterator for Iter<'_, T> {
    fn len(&self) -> usize {
        self.len as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lru_order() {
        let mut cache = LruSlab::new();
        let b = cache.insert('b');
        assert_eq!(cache.iter().collect::<String>(), "b");
        let _a = cache.insert('a');
        assert_eq!(cache.iter().collect::<String>(), "ab");
        let d = cache.insert('d');
        assert_eq!(cache.iter().collect::<String>(), "dab");
        let c = cache.insert('c');
        assert_eq!(cache.iter().collect::<String>(), "cdab");
        let e = cache.insert('e');
        assert_eq!(cache.iter().collect::<String>(), "ecdab");

        cache.get_mut(b);
        cache.get_mut(c);
        cache.get_mut(d);
        cache.get_mut(e);

        assert_eq!(cache.remove(cache.lru().unwrap()), 'a');
        assert_eq!(cache.remove(cache.lru().unwrap()), 'b');
        assert_eq!(cache.remove(cache.lru().unwrap()), 'c');
        assert_eq!(cache.remove(cache.lru().unwrap()), 'd');
        assert_eq!(cache.remove(cache.lru().unwrap()), 'e');
        assert!(cache.lru().is_none());
    }

    #[test]
    fn slot_reuse() {
        let mut cache = LruSlab::new();
        let a = cache.insert('a');
        cache.remove(a);
        let a_prime = cache.insert('a');
        assert_eq!(a, a_prime);
        assert_eq!(cache.len(), 1);
    }
}
