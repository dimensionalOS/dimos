include!(concat!(env!("OUT_DIR"), "/extras.rs"));

fn main() {
    println!("dimos {} ({} extras)", env!("DIMOS_VERSION"), EXTRAS.len());
}
