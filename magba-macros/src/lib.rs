/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input,
    punctuated::Punctuated,
    Expr, Result, Token, Type,
};

/// Custom parser for collection macro input
/// Supports two syntaxes:
/// 1. `collection![CylinderMagnet: source1, source2, source3]` - Homogeneous sources
/// 2. `collection![source1, source2, source3]` - Heterogeneous sources
struct CollectionInput {
    type_hint: Option<Type>,
    sources: Punctuated<Expr, Token![,]>,
}

impl Parse for CollectionInput {
    fn parse(input: ParseStream) -> Result<Self> {
        // Try to parse optional type hint followed by colon
        let type_hint = if input.peek2(Token![:]) {
            let ty = input.parse::<Type>()?;
            input.parse::<Token![:]>()?;
            Some(ty)
        } else {
            None
        };

        let sources = Punctuated::<Expr, Token![,]>::parse_terminated(input)?;

        Ok(CollectionInput { type_hint, sources })
    }
}

/// Creates a `Collection` based on source type homogeneity.
///
/// # Syntax
///
/// - **Empty homogeneous collection**: `collection![TypeName:]`
/// - **Homogeneous sources** (Collection):
///   `collection![TypeName: expr1, expr2, ...]`
/// - **Empty heterogeneous collection**: `collection![]`
/// - **Heterogeneous sources** (Collection):
///   `collection![expr1, expr2, ...]`
///
/// # Examples
///
/// ```ignore
/// use magba::*;
/// use nalgebra::*;
///
/// // Empty homogeneous collection
/// let mut coll = collection![CylinderMagnet:];
/// // Add sources later
/// coll.add(CylinderMagnet::default());
///
/// // Homogeneous collection (stack-allocated Collection)
/// let magnet1 = CylinderMagnet::default();
/// let magnet2 = CylinderMagnet::new(
///     Point3::new(0.0, 0.0, 1.0),
///     UnitQuaternion::identity(),
///     Vector3::z(),
///     0.1,
///     0.2,
/// );
/// let coll = collection![CylinderMagnet: magnet1, magnet2];
///
/// // Empty heterogeneous collection
/// let mut coll = collection![];
/// // Add sources later
/// coll.add(Box::new(CylinderMagnet::default()));
///
/// // Heterogeneous collection (heap-allocated Collection)
/// let cylinder = CylinderMagnet::default();
/// let dipole = Dipole::default();
/// let cuboid = CuboidMagnet::default();
/// let coll = collection![cylinder, dipole, cuboid];
/// ```
#[proc_macro]
pub fn collection(input: TokenStream) -> TokenStream {
    let CollectionInput { type_hint, sources } = parse_macro_input!(input as CollectionInput);

    let source_exprs: Vec<_> = sources.iter().collect();

    if let Some(source_type) = type_hint {
        // Explicit type hint provided - create homogeneous Collection (empty or with sources)
        // Always use f64 for float type
        quote! {
            {
                let sources: Vec<#source_type> = vec![#(#source_exprs),*];
                magba::Collection::<#source_type, f64>::from_sources(sources)
            }
        }
        .into()
    } else {
        // No type hint - create heterogeneous Collection (empty or with sources)
        // Each source is boxed for heap allocation with identity rotation at origin
        // Always use f64 for float type
        quote! {
            {
                let sources: Vec<Box<dyn magba::Source<f64>>> = vec![
                    #(Box::new(#source_exprs)),*
                ];
                magba::Collection::<Box<dyn magba::Source<f64>>, f64>::from_sources(sources)
            }
        }
        .into()
    }
}
