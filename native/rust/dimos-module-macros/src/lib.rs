// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{parse_macro_input, Data, DeriveInput, Field, Fields, Ident, LitStr, Path, Type};

#[proc_macro_derive(Module, attributes(input, output, io, config, tf, module))]
pub fn derive_module(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match expand(input) {
        Ok(ts) => ts.into(),
        Err(e) => e.to_compile_error().into(),
    }
}

/// Defines a native-module config: every field is required and supplied by the Python wrapper
/// over stdin, with no Rust-side defaults.
///
/// ```ignore
/// #[native_config]
/// pub struct Config {
///     #[validate(range(min = 0.0))]
///     pub voxel_size: f32,
/// }
/// ```
///
/// Injects `#[derive(Debug, Deserialize, Serialize, Validate)]` and
/// `#[serde(deny_unknown_fields)]`, and emits `impl NativeConfig`.
///
/// Rejected at compile time:
/// - `Option<T>` fields
/// - `#[serde(default)]`, field or container
/// - `#[serde(skip)]`, `#[serde(skip_deserializing)]`, `#[serde(flatten)]`
///
/// A type alias to `Option` is not caught here, but `run()` rejects a missing
/// field at startup regardless of how the field is spelled.
#[proc_macro_attribute]
pub fn native_config(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);
    let maybe_err = check_native_config(&input)
        .err()
        .map(|e| e.to_compile_error());

    // Emit the expansion with any error so a failed check shows our useful message.
    let injectable = matches!(
        &input.data,
        Data::Struct(s) if matches!(s.fields, Fields::Named(_) | Fields::Unit)
    );
    if !injectable {
        return quote!(#input #maybe_err).into();
    }

    let name = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();
    quote! {
        #[derive(Debug, ::serde::Deserialize, ::serde::Serialize, ::validator::Validate)]
        #[serde(deny_unknown_fields)]
        #input
        impl #impl_generics ::dimos_module::NativeConfig for #name #ty_generics #where_clause {}
        #maybe_err
    }
    .into()
}

fn check_native_config(input: &DeriveInput) -> syn::Result<()> {
    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            Fields::Unit => return check_container_serde(input),
            Fields::Unnamed(_) => {
                return Err(syn::Error::new_spanned(
                    input,
                    "native_config requires a struct with named fields or a unit struct",
                ))
            }
        },
        _ => {
            return Err(syn::Error::new_spanned(
                input,
                "native_config can only be applied to structs",
            ))
        }
    };

    check_container_serde(input)?;

    for field in fields {
        if is_option(&field.ty) {
            return Err(syn::Error::new_spanned(
                &field.ty,
                "native_config forbids Option fields: an absent field would silently become None. \
                 Make it required and let Python always send it.",
            ));
        }
        check_field_serde(field)?;
    }

    Ok(())
}

/// Reject a container-level `#[serde(default)]`. The macro injects
/// `deny_unknown_fields` itself.
fn check_container_serde(input: &DeriveInput) -> syn::Result<()> {
    for attr in &input.attrs {
        if !attr.path().is_ident("serde") {
            continue;
        }
        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("default") {
                return Err(meta.error(
                    "native_config forbids #[serde(default)]: Python owns defaults and sends \
                     every field. Remove the default and make every field required.",
                ));
            }
            // allow other serde args that take a `= value`
            consume_optional_value(&meta);
            Ok(())
        })?;
    }
    Ok(())
}

/// Reject field-level `#[serde(default | skip | skip_deserializing | flatten)]`.
fn check_field_serde(field: &Field) -> syn::Result<()> {
    for attr in &field.attrs {
        if !attr.path().is_ident("serde") {
            continue;
        }
        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("default") {
                return Err(meta.error(
                    "native_config forbids #[serde(default)]: Python owns defaults and sends \
                     every field. Make it required.",
                ));
            }
            if meta.path.is_ident("skip") || meta.path.is_ident("skip_deserializing") {
                return Err(meta.error(
                    "native_config forbids #[serde(skip)]: a skipped field is filled by Rust \
                     instead of Python.",
                ));
            }
            if meta.path.is_ident("flatten") {
                return Err(meta.error(
                    "native_config forbids #[serde(flatten)]: it bypasses deny_unknown_fields.",
                ));
            }
            consume_optional_value(&meta);
            Ok(())
        })?;
    }
    Ok(())
}

fn consume_optional_value(meta: &syn::meta::ParseNestedMeta) {
    if meta.input.peek(syn::Token![=]) {
        let _ = meta.value().and_then(|v| v.parse::<syn::Expr>());
    }
}

fn is_option(ty: &Type) -> bool {
    matches!(ty, Type::Path(p) if p.path.segments.last().is_some_and(|s| s.ident == "Option"))
}

const ONE_ATTR_ONLY: &str = "field has multiple module attributes; only one of #[input], \
                             #[output], #[io], #[config], #[tf] is allowed";

/// What a `#[tf]` port carries, for the Cargo.toml registry cross-check.
const TF_PAYLOAD_TYPE: &str = "TFMessage";

enum FieldKind {
    Input {
        decode: Path,
        handler: Ident,
    },
    Output {
        encode: Path,
    },
    Io {
        decode: Path,
        encode: Path,
        handler: Ident,
    },
    Config,
    Tf,
    State,
}

struct ClassifiedField<'a> {
    name: &'a Ident,
    ty: &'a Type,
    kind: FieldKind,
    /// The dimos message identity from `msg = "..."`, for a port whose wire
    /// payload and dimos message type differ.
    msg: Option<String>,
}

/// A port field's attribute, parsed.
struct FieldAttr {
    kind: FieldKind,
    msg: Option<String>,
}

fn expand(input: DeriveInput) -> syn::Result<TokenStream2> {
    let struct_name = &input.ident;

    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            _ => {
                return Err(syn::Error::new_spanned(
                    &input,
                    "Module requires a struct with named fields",
                ))
            }
        },
        _ => {
            return Err(syn::Error::new_spanned(
                &input,
                "Module can only be derived for structs",
            ))
        }
    };

    let mut setup_method: Option<Ident> = None;
    let mut teardown_method: Option<Ident> = None;
    let mut registry_name: Option<syn::LitStr> = None;
    for attr in &input.attrs {
        if attr.path().is_ident("module") {
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("setup") {
                    setup_method = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("teardown") {
                    teardown_method = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("name") {
                    registry_name = Some(meta.value()?.parse()?);
                } else {
                    return Err(meta.error(
                        "unrecognized #[module] argument; expected `setup = ...`, \
                         `teardown = ...` or `name = \"...\"`",
                    ));
                }
                Ok(())
            })?;
        }
    }

    let mut classified: Vec<ClassifiedField> = Vec::new();
    let mut config_seen: Option<&Ident> = None;

    for field in fields {
        let name = field.ident.as_ref().expect("named field has an identifier");
        let attr = classify_field(field, name)?;
        if matches!(attr.kind, FieldKind::Config) {
            if let Some(prev) = config_seen {
                return Err(syn::Error::new_spanned(
                    field,
                    format!(
                        "multiple #[config] fields (previous: `{prev}`); at most one is allowed"
                    ),
                ));
            }
            config_seen = Some(name);
        }
        classified.push(ClassifiedField {
            name,
            ty: &field.ty,
            kind: attr.kind,
            msg: attr.msg,
        });
    }

    if let Some(name) = &registry_name {
        check_registry_ports(name, &classified).map_err(|msg| {
            syn::Error::new_spanned(
                struct_name,
                format!(
                    "#[module(name = \"{}\")] does not match Cargo.toml: {msg}",
                    name.value()
                ),
            )
        })?;
    }

    // check_registry_ports reads Cargo.toml behind rustc's back, so cargo would
    // not rerun the check on a manifest edit. Embedding the manifest records it
    // in dep-info, making the cross-check hold on incremental builds too.
    let manifest_dep: TokenStream2 = if registry_name.is_some() {
        quote! {
            const _: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml"));
        }
    } else {
        quote!()
    };

    let config_type: Type = classified
        .iter()
        .find_map(|f| matches!(f.kind, FieldKind::Config).then(|| f.ty.clone()))
        .unwrap_or_else(|| syn::parse_quote!(::dimos_module::NoConfig));

    let config_param: TokenStream2 = if config_seen.is_some() {
        quote!(config)
    } else {
        quote!(_config)
    };

    let build_field_inits = classified.iter().map(|f| {
        let name = f.name;
        let name_str = name.to_string();
        match &f.kind {
            FieldKind::Input { decode, .. } => {
                quote!(#name: builder.input(#name_str, #decode))
            }
            FieldKind::Output { encode } => {
                quote!(#name: builder.output(#name_str, #encode))
            }
            FieldKind::Io { decode, encode, .. } => {
                quote!(#name: builder.io(#name_str, #decode, #encode))
            }
            FieldKind::Config => quote!(#name: config),
            FieldKind::Tf => quote!(#name: builder.tf()),
            FieldKind::State => quote!(#name: ::core::default::Default::default()),
        }
    });

    // Every port that receives messages gets an arm in the select! loop.
    let handled_fields: Vec<(&Ident, &Ident)> = classified
        .iter()
        .filter_map(|f| match &f.kind {
            FieldKind::Input { handler, .. } | FieldKind::Io { handler, .. } => {
                Some((f.name, handler))
            }
            _ => None,
        })
        .collect();

    let handle_body = if handled_fields.is_empty() {
        quote!(::std::future::pending::<()>().await)
    } else {
        let handle_arms = handled_fields.iter().map(|(name, handler)| {
            quote!(
                ::core::option::Option::Some(msg) = self.#name.recv() => {
                    self.#handler(msg).await
                }
            )
        });
        quote! {
            loop {
                ::tokio::select! {
                    #(#handle_arms,)*
                    else => break,
                }
            }
        }
    };

    let setup_impl = setup_method.map(|m| {
        quote! {
            async fn setup(&mut self) {
                self.#m().await
            }
        }
    });

    let teardown_impl = teardown_method.map(|m| {
        quote! {
            async fn teardown(&mut self) {
                self.#m().await
            }
        }
    });

    Ok(quote! {
        #manifest_dep

        impl ::dimos_module::Module for #struct_name {
            type Config = #config_type;

            fn build(
                builder: &mut ::dimos_module::Builder,
                #config_param: <Self as ::dimos_module::Module>::Config,
            ) -> Self {
                Self {
                    #(#build_field_inits,)*
                }
            }

            #setup_impl

            async fn handle(&mut self) {
                #handle_body
            }

            #teardown_impl
        }
    })
}

/// A port as the struct declares it: field name plus the dimos message type it
/// carries, which is the payload type unless `msg` names a different one.
#[derive(Debug, PartialEq, Eq)]
struct PortDecl {
    name: String,
    ty: String,
}

/// The last `::`-separated segment of the single generic argument of
/// `Input<T>`/`Output<T>`, e.g. `Input<lcm_msgs::sensor_msgs::PointCloud2>` -> `PointCloud2`.
fn port_payload_type(ty: &Type) -> Option<String> {
    let Type::Path(p) = ty else { return None };
    let segment = p.path.segments.last()?;
    let syn::PathArguments::AngleBracketed(args) = &segment.arguments else {
        return None;
    };
    let syn::GenericArgument::Type(Type::Path(inner)) = args.args.first()? else {
        return None;
    };
    Some(inner.path.segments.last()?.ident.to_string())
}

fn port_decls(classified: &[ClassifiedField], want_input: bool) -> Vec<PortDecl> {
    classified
        .iter()
        .filter(|f| match f.kind {
            FieldKind::Input { .. } => want_input,
            FieldKind::Output { .. } => !want_input,
            // A `#[tf]` field is a port like any other as far as the registry
            // goes: bake has to put the topic in the host's map or the module
            // refuses to start. It lists under `inputs` because the graph has
            // no io kind and a host mostly consumes tf.
            FieldKind::Tf => want_input,
            _ => false,
        })
        .map(|f| PortDecl {
            name: f.name.to_string(),
            // `Tf` carries no generic payload. The port is TFMessage by
            // construction.
            // An explicit `msg` wins: a view type's wire payload and the dimos
            // message it stands for are allowed to differ.
            ty: match f.kind {
                FieldKind::Tf => TF_PAYLOAD_TYPE.to_string(),
                _ => f
                    .msg
                    .clone()
                    .or_else(|| port_payload_type(f.ty))
                    .unwrap_or_default(),
            },
        })
        .collect()
}

/// Compare the struct's ports against `[package.metadata.dimos.module.<name>]`
/// in the crate's own Cargo.toml, which `dimos bake` reads to draw the graph
/// before anything is compiled. The two must not drift.
fn check_registry_ports(name: &syn::LitStr, classified: &[ClassifiedField]) -> Result<(), String> {
    if let Some(io) = classified
        .iter()
        .find(|f| matches!(f.kind, FieldKind::Io { .. }))
    {
        return Err(format!(
            "port `{}` is #[io], which the bake registry does not support yet; \
             split it into an #[input] and an #[output] field",
            io.name
        ));
    }
    let dir = std::env::var("CARGO_MANIFEST_DIR")
        .map_err(|_| "CARGO_MANIFEST_DIR is unset, cannot locate Cargo.toml".to_string())?;
    let path = std::path::Path::new(&dir).join("Cargo.toml");
    let src = std::fs::read_to_string(&path)
        .map_err(|e| format!("failed to read {}: {e}", path.display()))?;
    check_manifest_ports(
        &src,
        &name.value(),
        &port_decls(classified, true),
        &port_decls(classified, false),
    )
}

/// Manifest half of [`check_registry_ports`], split out so it can be tested
/// against fixture manifests.
fn check_manifest_ports(
    manifest: &str,
    module_name: &str,
    inputs: &[PortDecl],
    outputs: &[PortDecl],
) -> Result<(), String> {
    let manifest: toml::Value =
        toml::from_str(manifest).map_err(|e| format!("failed to parse Cargo.toml: {e}"))?;
    let entry = ["package", "metadata", "dimos", "module", module_name]
        .iter()
        .try_fold(&manifest, |value, key| value.get(key))
        .ok_or_else(|| {
            format!("no [package.metadata.dimos.module.{module_name}] section in Cargo.toml")
        })?;

    check_port_table(entry, "inputs", inputs)?;
    check_port_table(entry, "outputs", outputs)?;
    Ok(())
}

fn check_port_table(entry: &toml::Value, kind: &str, declared: &[PortDecl]) -> Result<(), String> {
    let empty = toml::value::Table::new();
    let table = match entry.get(kind) {
        Some(v) => v
            .as_table()
            .ok_or_else(|| format!("`{kind}` must be a table of port = \"msg.Type\""))?,
        None => &empty,
    };

    for port in declared {
        let Some(msg) = table.get(&port.name) else {
            return Err(format!("port `{}` is missing from `{kind}`", port.name));
        };
        let msg = msg
            .as_str()
            .ok_or_else(|| format!("`{kind}.{}` must be a string", port.name))?;
        let want = msg.rsplit('.').next().unwrap_or(msg);
        if want != port.ty {
            return Err(format!(
                "port `{}` is declared `{msg}` in `{kind}` but the struct field carries `{}`",
                port.name, port.ty
            ));
        }
    }

    for port in table.keys() {
        if !declared.iter().any(|d| &d.name == port) {
            return Err(format!(
                "`{kind}` declares port `{port}`, which the struct has no #[{}] field for",
                kind.trim_end_matches('s')
            ));
        }
    }
    Ok(())
}

fn classify_field(field: &Field, name: &Ident) -> syn::Result<FieldAttr> {
    let mut found: Option<FieldKind> = None;
    let mut msg: Option<String> = None;

    for attr in &field.attrs {
        let path = attr.path();
        if path.is_ident("input") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(attr, ONE_ATTR_ONLY));
            }
            let mut decode: Option<Path> = None;
            let mut handler: Option<Ident> = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("decode") {
                    decode = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("handler") {
                    handler = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("msg") {
                    msg = Some(meta.value()?.parse::<LitStr>()?.value());
                } else {
                    return Err(meta.error(
                        "unrecognized #[input] argument; expected `decode = ...`, \
                         `handler = ...` or `msg = ...`",
                    ));
                }
                Ok(())
            })?;
            let decode = decode
                .ok_or_else(|| syn::Error::new_spanned(attr, "#[input] requires `decode = ...`"))?;
            let handler = handler.unwrap_or_else(|| format_ident!("handle_{}", name));
            found = Some(FieldKind::Input { decode, handler });
        } else if path.is_ident("output") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(attr, ONE_ATTR_ONLY));
            }
            let mut encode: Option<Path> = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("encode") {
                    encode = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("msg") {
                    msg = Some(meta.value()?.parse::<LitStr>()?.value());
                } else {
                    return Err(meta.error(
                        "unrecognized #[output] argument; expected `encode = ...` or `msg = ...`",
                    ));
                }
                Ok(())
            })?;
            let encode = encode.ok_or_else(|| {
                syn::Error::new_spanned(attr, "#[output] requires `encode = ...`")
            })?;
            found = Some(FieldKind::Output { encode });
        } else if path.is_ident("io") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(attr, ONE_ATTR_ONLY));
            }
            let mut decode: Option<Path> = None;
            let mut encode: Option<Path> = None;
            let mut handler: Option<Ident> = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("decode") {
                    decode = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("encode") {
                    encode = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("handler") {
                    handler = Some(meta.value()?.parse()?);
                } else {
                    return Err(meta.error(
                        "unrecognized #[io] argument; expected `decode = ...`, `encode = ...` or `handler = ...`",
                    ));
                }
                Ok(())
            })?;
            let decode = decode
                .ok_or_else(|| syn::Error::new_spanned(attr, "#[io] requires `decode = ...`"))?;
            let encode = encode
                .ok_or_else(|| syn::Error::new_spanned(attr, "#[io] requires `encode = ...`"))?;
            let handler = handler.unwrap_or_else(|| format_ident!("handle_{}", name));
            found = Some(FieldKind::Io {
                decode,
                encode,
                handler,
            });
        } else if path.is_ident("config") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(attr, ONE_ATTR_ONLY));
            }
            found = Some(FieldKind::Config);
        } else if path.is_ident("tf") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(attr, ONE_ATTR_ONLY));
            }
            found = Some(FieldKind::Tf);
        }
    }

    Ok(FieldAttr {
        kind: found.unwrap_or(FieldKind::State),
        msg,
    })
}

#[cfg(test)]
mod tests {
    use super::check_native_config;
    use syn::parse_str;

    fn check(src: &str) -> syn::Result<()> {
        check_native_config(&parse_str(src).expect("test input should parse"))
    }

    #[test]
    fn accepts_plain_required_fields() {
        check(r#"struct Config { a: f32, b: String, c: u32 }"#)
            .expect("a plain required-field struct should pass");
    }

    #[test]
    fn accepts_unit_struct() {
        check(r#"struct NoConfig;"#).expect("a field-less struct should pass");
    }

    #[test]
    fn accepts_validate_attrs() {
        check(r#"struct Config { #[validate(range(min = 0))] a: i64 }"#)
            .expect("validate attrs should pass through");
    }

    #[test]
    fn rejects_option_field() {
        let err =
            check(r#"struct Config { a: Option<f32> }"#).expect_err("Option fields are forbidden");
        assert!(err.to_string().contains("Option"), "{err}");
    }

    #[test]
    fn rejects_field_default() {
        check(r#"struct Config { #[serde(default)] a: f32 }"#)
            .expect_err("field #[serde(default)] is forbidden");
    }

    #[test]
    fn rejects_field_default_with_path() {
        check(r#"struct Config { #[serde(default = "mk")] a: f32 }"#)
            .expect_err("#[serde(default = ...)] is forbidden");
    }

    #[test]
    fn rejects_container_default() {
        check(r#"#[serde(default)] struct Config { a: f32 }"#)
            .expect_err("container #[serde(default)] is forbidden");
    }

    #[test]
    fn rejects_flatten_and_skip() {
        check(r#"struct Config { #[serde(flatten)] a: Inner }"#).expect_err("flatten is forbidden");
        check(r#"struct Config { #[serde(skip)] a: f32 }"#).expect_err("skip is forbidden");
        check(r#"struct Config { #[serde(skip_deserializing)] a: f32 }"#)
            .expect_err("skip_deserializing is forbidden");
    }

    #[test]
    fn rejects_tuple_struct() {
        check(r#"struct Config(f32);"#).expect_err("tuple structs are not valid configs");
    }

    #[test]
    fn rejects_enum() {
        check(r#"enum Config { A, B }"#).expect_err("enums are not valid configs");
    }

    // registry metadata cross-check

    use super::{
        check_manifest_ports, check_registry_ports, port_payload_type, ClassifiedField, FieldKind,
        PortDecl,
    };
    use syn::parse_quote;

    #[test]
    fn io_ports_are_rejected_by_the_registry_check() {
        let name: syn::LitStr = parse_quote!("demo");
        let ident: syn::Ident = parse_quote!(cmd);
        let ty: syn::Type = parse_quote!(Io<Twist>);
        let classified = [ClassifiedField {
            name: &ident,
            ty: &ty,
            kind: FieldKind::Io {
                decode: parse_quote!(demo::decode),
                encode: parse_quote!(demo::encode),
                handler: parse_quote!(on_cmd),
            },
            msg: None,
        }];
        let err = check_registry_ports(&name, &classified).expect_err("io ports are unsupported");
        assert!(err.contains("cmd"), "{err}");
        assert!(err.contains("#[io]"), "{err}");
    }

    const MANIFEST: &str = r#"
        [package]
        name = "demo"

        [package.metadata.dimos.module.demo]
        path = "demo::module::Demo"
        python = "dimos.demo.module:Demo"
        threads = 1

        [package.metadata.dimos.module.demo.inputs]
        lidar = "sensor_msgs.PointCloud2"

        [package.metadata.dimos.module.demo.outputs]
        global_map = "sensor_msgs.PointCloud2"
    "#;

    fn port(name: &str, ty: &str) -> PortDecl {
        PortDecl {
            name: name.to_string(),
            ty: ty.to_string(),
        }
    }

    #[test]
    fn accepts_manifest_matching_the_struct() {
        check_manifest_ports(
            MANIFEST,
            "demo",
            &[port("lidar", "PointCloud2")],
            &[port("global_map", "PointCloud2")],
        )
        .expect("matching ports should pass");
    }

    #[test]
    fn rejects_missing_metadata_section() {
        let err = check_manifest_ports(MANIFEST, "other", &[], &[])
            .expect_err("an unregistered module id must fail");
        assert!(err.contains("package.metadata.dimos.module.other"), "{err}");
    }

    #[test]
    fn rejects_port_absent_from_manifest() {
        let err = check_manifest_ports(
            MANIFEST,
            "demo",
            &[port("lidar", "PointCloud2"), port("odometry", "Odometry")],
            &[port("global_map", "PointCloud2")],
        )
        .expect_err("a struct port missing from the manifest must fail");
        assert!(err.contains("odometry"), "{err}");
    }

    #[test]
    fn rejects_port_absent_from_struct() {
        let err = check_manifest_ports(MANIFEST, "demo", &[], &[port("global_map", "PointCloud2")])
            .expect_err("a manifest port with no struct field must fail");
        assert!(err.contains("lidar"), "{err}");
    }

    #[test]
    fn rejects_msg_type_mismatch() {
        let err = check_manifest_ports(
            MANIFEST,
            "demo",
            &[port("lidar", "Odometry")],
            &[port("global_map", "PointCloud2")],
        )
        .expect_err("a differing payload type must fail");
        assert!(
            err.contains("lidar") && err.contains("PointCloud2"),
            "{err}"
        );
    }

    /// The package half is checked against the python wrapper by bake, in
    /// `test_every_baked_port_lands_on_the_key_its_python_wrapper_subscribes_to`.
    #[test]
    fn compares_only_the_last_msg_type_segment() {
        // The manifest spells the package. The struct spells the imported name.
        let manifest = MANIFEST.replace(
            "lidar = \"sensor_msgs.PointCloud2\"",
            "lidar = \"other_msgs.PointCloud2\"",
        );
        check_manifest_ports(
            &manifest,
            "demo",
            &[port("lidar", "PointCloud2")],
            &[port("global_map", "PointCloud2")],
        )
        .expect("package-qualified manifest types match bare struct types");
    }

    /// A view type's wire payload and the dimos message it stands for differ,
    /// so the port declares the message and the manifest must match that.
    #[test]
    fn an_explicit_msg_names_the_dimos_message_type() {
        let manifest = MANIFEST.replace(
            "global_map = \"sensor_msgs.PointCloud2\"",
            "global_map = \"nav_msgs.LineSegments3D\"",
        );
        check_manifest_ports(
            &manifest,
            "demo",
            &[port("lidar", "PointCloud2")],
            &[port("global_map", "LineSegments3D")],
        )
        .expect("a declared msg type matches the manifest");
    }

    #[test]
    fn payload_type_is_the_last_generic_segment() {
        let ty: syn::Type = parse_str("Input<lcm_msgs::sensor_msgs::PointCloud2>").unwrap();
        assert_eq!(port_payload_type(&ty).as_deref(), Some("PointCloud2"));
        let ty: syn::Type = parse_str("Output<Path>").unwrap();
        assert_eq!(port_payload_type(&ty).as_deref(), Some("Path"));
        let ty: syn::Type = parse_str("Input").unwrap();
        assert_eq!(port_payload_type(&ty), None);
    }
}
