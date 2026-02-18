/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::magnets::define_magnet::define_magnet;

define_magnet! {
    ZeroMagnet
    field_fn: zero_field
    args: {}
    arg_display: "Placeholder";
    arg_fmt: []
}
