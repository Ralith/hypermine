/// A fixed-size table of handles with constant-time insert and free of least recently used value
pub struct LruTable<T> {
    slots: Box<[Slot<T>]>,
    free: Vec<SlotId>,
    head: SlotId,
    tail: SlotId,
}

impl<T> LruTable<T> {
    pub fn with_capacity(n: u32) -> Self {
        assert!(n != u32::max_value(), "capacity too large");
        Self {
            slots: (0..n)
                .map(|_| Slot {
                    value: None,
                    prev: SlotId::NONE,
                    next: SlotId::NONE,
                })
                .collect::<Vec<_>>()
                .into(),
            free: (0..n).map(SlotId).collect(),
            head: SlotId::NONE,
            tail: SlotId::NONE,
        }
    }

    /// Whether `insert` is guaranteed to return `None`
    pub fn is_full(&self) -> bool {
        self.free.is_empty()
    }

    pub fn capacity(&self) -> u32 {
        self.slots.len() as u32
    }

    /// Inserts a value, returning the slot it was stored in
    ///
    /// The returned slot is marked as the most recently used.
    pub fn insert(&mut self, value: T) -> Option<SlotId> {
        let id = self.free.pop()?;
        let idx = id.0 as usize;

        debug_assert!(self.slots[idx].value.is_none(), "corrupt lru list");
        self.slots[idx].value = Some(value);

        debug_assert_eq!(self.slots[idx].next, SlotId::NONE, "corrupt lru list");
        debug_assert_eq!(self.slots[idx].prev, SlotId::NONE, "corrupt lru list");
        self.link_at_head(id);

        Some(id)
    }

    /// Get the value in the least recently used slot
    pub fn peek_lru(&self) -> Option<&T> {
        if self.tail == SlotId::NONE {
            // Empty
            return None;
        }
        Some(
            self.slots[self.tail.0 as usize]
                .value
                .as_ref()
                .expect("empty slot in lru list"),
        )
    }

    /// Free the least recently used slot
    pub fn remove_lru(&mut self) -> Option<T> {
        let slot = self.tail;
        if slot == SlotId::NONE {
            debug_assert_eq!(self.free.len(), self.slots.len());
            // Already empty
            return None;
        }
        self.unlink(slot);
        self.free.push(slot);
        // To help make corruption obvious, empty slots have no next/prev
        self.slots[slot.0 as usize].prev = SlotId::NONE;
        Some(
            self.slots[slot.0 as usize]
                .value
                .take()
                .expect("tried to free empty slot"),
        )
    }

    /// Mark `slot` as the most recently used and access it uniquely
    pub fn get_mut(&mut self, slot: SlotId) -> &mut T {
        self.freshen(slot);
        self.peek_mut(slot)
    }

    /// Access `slot` uniquely without marking it as most recently used
    pub fn peek_mut(&mut self, slot: SlotId) -> &mut T {
        self.slots[slot.0 as usize].value.as_mut().unwrap()
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

struct Slot<T> {
    value: Option<T>,
    next: SlotId,
    prev: SlotId,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct SlotId(pub u32);

impl SlotId {
    const NONE: Self = SlotId(u32::max_value());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lru_order() {
        let mut cache = LruTable::with_capacity(4);
        let b = cache.insert(1).unwrap();
        let _a = cache.insert(0).unwrap();
        let d = cache.insert(3).unwrap();
        let c = cache.insert(2).unwrap();

        assert!(cache.insert(42).is_none());

        cache.get_mut(b);
        cache.get_mut(c);
        cache.get_mut(d);

        assert_eq!(cache.remove_lru().unwrap(), 0);
        assert_eq!(cache.remove_lru().unwrap(), 1);
        assert_eq!(cache.remove_lru().unwrap(), 2);
        assert_eq!(cache.remove_lru().unwrap(), 3);
        assert!(cache.remove_lru().is_none());
    }
}
