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
        .unwrap_or_else(|| syn::parse_quote!(()));

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
