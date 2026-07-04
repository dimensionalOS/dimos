use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{parse_macro_input, Data, DeriveInput, Field, Fields, Ident, Path, Type};

#[proc_macro_derive(Module, attributes(input, output, config, module))]
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

enum FieldKind {
    Input { decode: Path, handler: Ident },
    Output { encode: Path },
    Config,
    State,
}

struct ClassifiedField<'a> {
    name: &'a Ident,
    ty: &'a Type,
    kind: FieldKind,
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
    for attr in &input.attrs {
        if attr.path().is_ident("module") {
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("setup") {
                    setup_method = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("teardown") {
                    teardown_method = Some(meta.value()?.parse()?);
                } else {
                    return Err(meta.error(
                        "unrecognized #[module] argument; expected `setup = ...` or `teardown = ...`",
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
        let kind = classify_field(field, name)?;
        if matches!(kind, FieldKind::Config) {
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
            kind,
        });
    }

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
            FieldKind::Config => quote!(#name: config),
            FieldKind::State => quote!(#name: ::core::default::Default::default()),
        }
    });

    let input_fields: Vec<&ClassifiedField> = classified
        .iter()
        .filter(|f| matches!(f.kind, FieldKind::Input { .. }))
        .collect();

    let handle_body = if input_fields.is_empty() {
        quote!(::std::future::pending::<()>().await)
    } else {
        let handle_arms = input_fields.iter().map(|f| {
            let FieldKind::Input { handler, .. } = &f.kind else {
                unreachable!()
            };
            let name = f.name;
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

fn classify_field(field: &Field, name: &Ident) -> syn::Result<FieldKind> {
    let mut found: Option<FieldKind> = None;

    for attr in &field.attrs {
        let path = attr.path();
        if path.is_ident("input") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(
                    attr,
                    "field has multiple module attributes; only one of #[input], #[output], #[config] is allowed",
                ));
            }
            let mut decode: Option<Path> = None;
            let mut handler: Option<Ident> = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("decode") {
                    decode = Some(meta.value()?.parse()?);
                } else if meta.path.is_ident("handler") {
                    handler = Some(meta.value()?.parse()?);
                } else {
                    return Err(meta.error(
                        "unrecognized #[input] argument; expected `decode = ...` or `handler = ...`",
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
                return Err(syn::Error::new_spanned(
                    attr,
                    "field has multiple module attributes; only one of #[input], #[output], #[config] is allowed",
                ));
            }
            let mut encode: Option<Path> = None;
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("encode") {
                    encode = Some(meta.value()?.parse()?);
                } else {
                    return Err(
                        meta.error("unrecognized #[output] argument; expected `encode = ...`")
                    );
                }
                Ok(())
            })?;
            let encode = encode.ok_or_else(|| {
                syn::Error::new_spanned(attr, "#[output] requires `encode = ...`")
            })?;
            found = Some(FieldKind::Output { encode });
        } else if path.is_ident("config") {
            if found.is_some() {
                return Err(syn::Error::new_spanned(
                    attr,
                    "field has multiple module attributes; only one of #[input], #[output], #[config] is allowed",
                ));
            }
            found = Some(FieldKind::Config);
        }
    }

    Ok(found.unwrap_or(FieldKind::State))
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
}
