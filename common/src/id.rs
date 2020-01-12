#[macro_export]
macro_rules! mkid {
    ($name:ident : $ty:ty) => {
        #[derive(Debug, Eq, PartialEq, Ord, PartialOrd, Copy, Clone, Hash)]
        pub struct $name($ty);

        impl From<$ty> for $name {
            fn from(x: $ty) -> $name {
                $name(x)
            }
        }

        impl From<$name> for $ty {
            fn from(x: $name) -> $ty {
                x.0
            }
        }

        impl std::str::FromStr for $name {
            type Err = ::std::num::ParseIntError;
            fn from_str(s: &str) -> Result<Self, Self::Err> {
                Ok($name(<$ty>::from_str_radix(s, 16)?))
            }
        }

        impl serde::Serialize for $name {
            fn serialize<S>(&self, s: S) -> Result<S::Ok, S::Error>
            where
                S: serde::Serializer,
            {
                if s.is_human_readable() {
                    s.serialize_str(&self.to_string())
                } else {
                    self.0.serialize(s)
                }
            }
        }

        impl<'a> serde::Deserialize<'a> for $name {
            fn deserialize<D>(d: D) -> Result<Self, D::Error>
            where
                D: serde::Deserializer<'a>,
            {
                use serde::de::Error;
                if d.is_human_readable() {
                    let x = <&'a str>::deserialize(d)?;
                    x.parse().map_err(D::Error::custom)
                } else {
                    Ok($name(<$ty>::deserialize(d)?))
                }
            }
        }
    };
}
