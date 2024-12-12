use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

#[proc_macro_attribute]
pub fn add_cycles(attr: TokenStream, item: TokenStream) -> TokenStream {
    let cycles = parse_macro_input!(attr as syn::LitInt);
    let input_fn = parse_macro_input!(item as ItemFn);
    
    let vis = &input_fn.vis;
    let sig = &input_fn.sig;
    let block = &input_fn.block;
    
    let expanded = quote! {
        #vis #sig {
            crate::mmu::add_cycles(n64, #cycles);
            #block
        }
    };
    expanded.into()
}

#[proc_macro_attribute]
pub fn add_rdram_cycles(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    
    let vis = &input_fn.vis;
    let sig = &input_fn.sig;
    let block = &input_fn.block;
    
    let expanded = quote! {
        #vis #sig {
            crate::mmu::add_cycles(n64, 31 + (32 / 3));
            #block
        }
    };
    expanded.into()
}

