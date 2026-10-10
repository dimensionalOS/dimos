/*
    Copyright 2021 Sojan James

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

// Rust deserializer for CycloneDDS.
// See discussion at https://github.com/eclipse-cyclonedds/cyclonedds/issues/830

use cdr::{Bounded, CdrBe, Infinite};

use serde::{de::DeserializeOwned, Serialize};
use std::io::prelude::*;

use std::ptr::NonNull;

use std::{
    ffi::{c_void, CStr},
    marker::PhantomData,
    ops::Deref,
    sync::Arc,
};

use cyclonedds_sys::*;
//use fasthash::{murmur3::Hasher32, FastHasher};
use murmur3::murmur3_32;
use std::io::Cursor;

#[repr(C)]
pub struct SerType<T> {
    sertype: ddsi_sertype,
    _phantom: PhantomData<T>,
}

pub trait TopicType: Serialize + DeserializeOwned {
    // generate a non-cryptographic hash of the key values to be used internally
    // in cyclonedds
    fn hash(&self, basehash: u32) -> u32 {
        let cdr = self.key_cdr();
        let mut cursor = Cursor::new(cdr.as_slice());
        murmur3_32(&mut cursor, 0).unwrap() ^ basehash
    }

    fn is_fixed_size() -> bool {
        false
    }
    /// The type name for this topic
    fn typename() -> std::ffi::CString {
        let ty_name_parts: String = std::any::type_name::<Self>()
            .split("::")
            .skip(1)
            .collect::<Vec<_>>()
            .join("::");

        //println!("Typename:{:?}", &typename);
        std::ffi::CString::new(ty_name_parts).expect("Unable to create CString for type name")
    }

    /// The default topic_name to use when creating a topic of this type. The default
    /// implementation uses '/' instead of '::' to form a unix like path.
    /// A prefix can optionally be added
    fn topic_name(maybe_prefix: Option<&str>) -> String {
        let topic_name_parts: String = format!(
            "/{}",
            std::any::type_name::<Self>()
                .to_string()
                .split("::")
                .skip(1)
                .collect::<Vec<_>>()
                .join("/")
        );

        if let Some(prefix) = maybe_prefix {
            let mut path = String::from(prefix);
            path.push_str(&topic_name_parts);
            path
        } else {
            topic_name_parts
        }
    }

    fn has_key() -> bool;
    // this is the key as defined in the DDS-RTPS spec.
    // KeyHash (PID_KEY_HASH). This function does not
    // hash the key. Use the force_md5_keyhash to know
    // whether to use md5 even if the the key cdr is 16 bytes
    // or shorter.
    fn key_cdr(&self) -> Vec<u8>;

    // force the use of md5 even if the serialized size is less than 16
    // as per the standard, we need to check the potential field size and not the actual.
    fn force_md5_keyhash() -> bool;
}

impl<'a, T> SerType<T> {
    pub fn new() -> Box<SerType<T>>
    where
        T: DeserializeOwned + Serialize + TopicType,
    {
        Box::<SerType<T>>::new(SerType {
            sertype: {
                let mut sertype = std::mem::MaybeUninit::uninit();
                unsafe {
                    let type_name = T::typename();
                    ddsi_sertype_init(
                        sertype.as_mut_ptr(),
                        type_name.as_ptr(),
                        Box::into_raw(create_sertype_ops::<T>()),
                        Box::into_raw(create_serdata_ops::<T>()),
                        !T::has_key(),
                    );
                    let mut sertype = sertype.assume_init();
                    sertype.set_fixed_size(if T::is_fixed_size() { 1 } else { 0 });
                    #[cfg(feature = "shm")]
                    {
                        sertype.iox_size = std::mem::size_of::<T>() as u32;
                    }
                    sertype
                }
            },
            _phantom: PhantomData,
        })
    }

    // cast into cyclone dds sertype.  Rust relinquishes ownership here.
    // Cyclone DDS will free this. But if you need to free this pointer
    // before handing it over to cyclone, make sure you explicitly free it
    pub fn into_sertype(sertype: Box<SerType<T>>) -> *mut ddsi_sertype {
        Box::<SerType<T>>::into_raw(sertype) as *mut ddsi_sertype
    }

    pub fn try_from_sertype(sertype: *const ddsi_sertype) -> Option<Box<SerType<T>>> {
        let ptr = sertype as *mut SerType<T>;
        if !ptr.is_null() {
            Some(unsafe { Box::from_raw(ptr) })
        } else {
            None
        }
    }
}

#[derive(Clone)]
pub enum SampleStorage<T> {
    Owned(Arc<T>),
    Loaned(Arc<NonNull<T>>),
}

impl<T> Deref for SampleStorage<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            SampleStorage::Owned(t) => t.deref(),
            SampleStorage::Loaned(t) => unsafe { t.as_ref().as_ref() },
        }
    }
}

impl<T> Drop for SampleStorage<T> {
    fn drop(&mut self) {
        match self {
            SampleStorage::Loaned(_t) => {}
            _ => {}
        }
    }
}

pub struct Sample<T> {
    //Serdata is used for incoming samples. We hold a reference to the ddsi_serdata which contains
    // the sample
    serdata: Option<*mut ddsi_serdata>,
    // sample is used for outgoing samples.
    sample: Option<SampleStorage<T>>,
}

impl<'a, T> Sample<T>
where
    T: TopicType,
{
    pub fn try_deref(&self) -> Option<&T> {
        if let Some(serdata) = self.serdata {
            let serdata = SerData::<T>::mut_ref_from_serdata(serdata);
            match &serdata.sample {
                SampleData::Uninitialized => None,
                SampleData::SDKKey => None,
                SampleData::SDKData(it) => Some(it.as_ref()),
                SampleData::SHMData(it) => unsafe { Some(it.as_ref()) },
            }
        } else {
            None
        }
    }

    pub fn get_sample(&self) -> Option<SampleStorage<T>> {
        //if let Ok(t) = self.sample.write() {
        match self.sample.as_ref() {
            Some(s) => match s {
                SampleStorage::Owned(s) => Some(SampleStorage::Owned(s.clone())),
                SampleStorage::Loaned(s) => Some(SampleStorage::Loaned(s.clone())),
            },
            None => None,
        }
    }

    // Deprecated as this function can panic
    #[deprecated]
    pub(crate) fn get(&self) -> Option<Arc<T>> {
        //let t = self.sample;
        match &self.sample {
            Some(SampleStorage::Owned(t)) => Some(t.clone()),
            Some(SampleStorage::Loaned(_t)) => None,
            None => None,
        }
    }

    pub(crate) fn set_serdata(&mut self, serdata: *mut ddsi_serdata) {
        // Increment the reference count
        unsafe {
            ddsi_serdata_addref(serdata);
        }
        // Release the ref this slot was already holding. SampleBuffer slots are
        // reused across takes and cyclone overwrites them through the C API, so
        // Sample::drop never runs for the displaced sample -- without this every
        // take leaks a whole serdata (payload included). Addref first, so
        // re-setting the same pointer can't transiently drop it to zero.
        if let Some(old) = self.serdata.replace(serdata) {
            unsafe {
                ddsi_serdata_removeref(old);
            }
        }
    }

    pub fn set(&mut self, t: Arc<T>) {
        //let mut sample = self.sample.write().unwrap();
        self.sample.replace(SampleStorage::Owned(t));
    }

    pub fn set_loaned(&mut self, t: NonNull<T>) {
        //let mut sample = self.sample.write().unwrap();
        self.sample.replace(SampleStorage::Loaned(Arc::new(t)));
    }

    pub fn clear(&mut self) {
        //let mut sample = self.sample.write().unwrap();
        let t = self.sample.take();

        match &t {
            Some(SampleStorage::Owned(_o)) => {}
            Some(SampleStorage::Loaned(_o)) => {}
            None => {}
        }
    }

    pub fn from(it: Arc<T>) -> Self {
        Self {
            serdata: None,
            sample: Some(SampleStorage::Owned(it)),
        }
    }
}

impl<T> Default for Sample<T> {
    fn default() -> Self {
        Self {
            serdata: None,
            sample: None,
        }
    }
}

impl<T> Drop for Sample<T> {
    fn drop(&mut self) {
        if let Some(serdata) = self.serdata {
            unsafe { ddsi_serdata_removeref(serdata) };
        }
    }
}

///
/// TODO: UNSAFE WARNING Review needed. Forcing SampleBuffer<T> to be Send
/// DDS read API uses an array of void* pointers. The SampleBuffer<T> structure
/// is used to create the sample array in the necessary format.
/// We allocate the Sample<T> structure and set it to deallocated here.
/// Cyclone does not allocate the sample, it only sets the value of the Arc<T>
/// inside the Sample<T>::Value<Arc<T>>.
/// So this structure always points to a valid sample memory, but the serdes callbacks
/// can change the value of the sample under us.
/// To be absolutely sure, I think we must put each sample into an RwLock<Arc<T>> instead of
/// an Arc<T>, I guess this is the cost we pay for zero copy.

unsafe impl<T> Send for SampleBuffer<T> {}
pub struct SampleBuffer<T> {
    /// This is !Send. This is the only way to punch through the Cyclone API as we need an array of pointers
    pub(crate) buffer: Vec<*mut Sample<T>>,
    pub(crate) sample_info: Vec<cyclonedds_sys::dds_sample_info>,
}

impl<'a, T: TopicType> SampleBuffer<T> {
    pub fn new(len: usize) -> Self {
        let mut buf = Self {
            buffer: Vec::new(),
            sample_info: vec![cyclonedds_sys::dds_sample_info::default(); len],
        };

        for _i in 0..len {
            let p = Box::into_raw(Box::default());
            buf.buffer.push(p);
        }
        buf
    }

    /// Check if sample is valid. Will panic if out of
    /// bounds.
    pub fn is_valid_sample(&self, index: usize) -> bool {
        self.sample_info[index].valid_data
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn iter(&'a self) -> impl Iterator<Item = &T> {
        let p = self.buffer.iter().filter_map(|p| {
            let sample = unsafe { &*(*p) };
            sample.try_deref()
        });
        p
    }

    /// Get a sample
    pub fn get(&self, index: usize) -> &Sample<T> {
        let p_sample = self.buffer[index];
        unsafe { &*p_sample }
    }

    /// return a raw pointer to the buffer and the sample info
    /// to be used in unsafe code that calls the CycloneDDS
    /// API
    pub unsafe fn as_mut_ptr(&mut self) -> (*mut *mut Sample<T>, *mut dds_sample_info) {
        (self.buffer.as_mut_ptr(), self.sample_info.as_mut_ptr())
    }
}

impl<'a, T> Drop for SampleBuffer<T> {
    fn drop(&mut self) {
        for p in &self.buffer {
            unsafe {
                let _it = Box::from_raw(*p);
            }
        }
    }
}
/*
impl <'a,T>Index<usize> for SampleBuffer<T> {
    type Output = &'a Sample<T>;
    fn index<'a>(&'a self, i: usize) -> &'a Sample<T> {
        &self.e[i]
    }
}
*/

#[allow(dead_code)]
unsafe extern "C" fn zero_samples<T>(
    _sertype: *const ddsi_sertype,
    _ptr: *mut std::ffi::c_void,
    _len: size_t,
) {
} // empty implementation

#[allow(dead_code)]
extern "C" fn realloc_samples<T>(
    ptrs: *mut *mut std::ffi::c_void,
    _sertype: *const ddsi_sertype,
    old: *mut std::ffi::c_void,
    old_count: size_t,
    new_count: size_t,
) {
    //println!("realloc");
    let old = unsafe {
        Vec::<*mut Sample<T>>::from_raw_parts(
            old as *mut *mut Sample<T>,
            old_count as usize,
            old_count as usize,
        )
    };
    let mut new = Vec::<*mut Sample<T>>::with_capacity(new_count as usize);

    if new_count >= old_count {
        for entry in old {
            new.push(entry);
        }

        for _i in 0..(new_count - old_count) {
            new.push(Box::into_raw(Box::default()));
        }
    } else {
        for e in old.into_iter().take(new_count as usize) {
            new.push(e)
        }
    }

    let leaked = new.leak();

    let (raw, _length) = (leaked.as_ptr(), leaked.len());
    // if the length and allocated length are not equal, we messed up above.
    //assert_eq!(length, allocated_length);
    unsafe {
        *ptrs = raw as *mut std::ffi::c_void;
    }
}

#[allow(dead_code)]
extern "C" fn free_samples<T>(
    _sertype: *const ddsi_sertype,
    ptrs: *mut *mut std::ffi::c_void,
    len: size_t,
    op: dds_free_op_t,
) where
    T: TopicType,
{
    let ptrs_v: *mut *mut Sample<T> = ptrs as *mut *mut Sample<T>;

    if (op & DDS_FREE_ALL_BIT) != 0 {
        let _samples =
            unsafe { Vec::<Sample<T>>::from_raw_parts(*ptrs_v, len as usize, len as usize) };
        // all samples will get freed when samples goes out of scope
    } else {
        assert_ne!(op & DDS_FREE_CONTENTS_BIT, 0);
        let mut samples =
            unsafe { Vec::<Sample<T>>::from_raw_parts(*ptrs_v, len as usize, len as usize) };
        for sample in samples.iter_mut() {
            //let _old_sample = std::mem::take(sample);
            sample.clear()
            //_old_sample goes out of scope and the content is freed. The pointer is replaced with a default constructed sample
        }
        let _intentional_leak = samples.leak();
    }
}

#[allow(dead_code)]
unsafe extern "C" fn free_sertype<T>(sertype: *mut cyclonedds_sys::ddsi_sertype) {
    ddsi_sertype_fini(sertype);

    let _sertype_ops = Box::<ddsi_sertype_ops>::from_raw((*sertype).ops as *mut ddsi_sertype_ops);
    let _serdata_ops =
        Box::<ddsi_serdata_ops>::from_raw((*sertype).serdata_ops as *mut ddsi_serdata_ops);
    // this sertype is always constructed in Rust. During destruction,
    // the Box takes over the pointer and frees it when it goes out
    // of scope.
    let sertype = sertype as *mut SerType<T>;
    let _it = Box::<SerType<T>>::from_raw(sertype);
}

// create ddsi_serdata from a fragchain
#[allow(dead_code)]
unsafe extern "C" fn serdata_from_fragchain<T>(
    sertype: *const ddsi_sertype,
    kind: u32,
    mut fragchain: *const nn_rdata,
    size: size_t,
) -> *mut ddsi_serdata
where
    T: DeserializeOwned + TopicType,
{
    //println!("serdata_from_fragchain");
    let mut off: u32 = 0;
    let size = size as usize;
    let fragchain_ref = &*fragchain;

    let mut serdata = SerData::<T>::new(sertype, kind);

    assert_eq!(fragchain_ref.min, 0);
    assert!(fragchain_ref.maxp1 >= off);

    // The scatter gather list
    let mut sg_list = Vec::new();

    while !fragchain.is_null() {
        let fragchain_ref = &*fragchain;
        if fragchain_ref.maxp1 > off {
            let payload =
                nn_rmsg_payload_offset(fragchain_ref.rmsg, nn_rdata_payload_offset(fragchain));
            let src = payload.add((off - fragchain_ref.min) as usize);
            let n_bytes = fragchain_ref.maxp1 - off;
            sg_list.push(std::slice::from_raw_parts(src, n_bytes as usize));
            off = fragchain_ref.maxp1;
            assert!(off as usize <= size);
        }
        fragchain = fragchain_ref.nextfrag;
    }
    // make a reader out of the sg_list
    let reader = SGReader::new(&sg_list);
    if let Ok(decoded) = cdr::deserialize_from::<_, T, _>(reader, Bounded(size as u64)) {
        if T::has_key() {
            // compute the 16byte key hash
            let key_cdr = decoded.key_cdr();
            // skip the four byte header
            let key_cdr = &key_cdr[4..];
            compute_key_hash(key_cdr, &mut serdata);
        }
        serdata.serdata.hash = decoded.hash((*sertype).serdata_basehash);
        let sample = std::sync::Arc::new(decoded);
        //store the deserialized sample in the serdata. We don't need to deserialize again
        serdata.sample = SampleData::SDKData(sample);
    } else {
        println!("Deserialization error!");
        return std::ptr::null_mut();
    }

    //store the hash into the serdata

    // convert into raw pointer and forget about it (for now). Cyclone will take ownership.
    let ptr = Box::into_raw(serdata);
    // only we know this ddsi_serdata is really of type SerData
    ptr as *mut ddsi_serdata
}

fn copy_raw_key_hash<T>(key: &[u8], serdata: &mut Box<SerData<T>>) {
    let mut raw_key = [0u8; 16];
    for (i, data) in key.iter().enumerate() {
        raw_key[i] = *data;
    }
    serdata.key_hash = KeyHash::RawKey(raw_key)
}

fn compute_key_hash<T>(key_cdr: &[u8], serdata: &mut SerData<T>)
where
    T: TopicType,
{
    let mut cdr_key = [0u8; 20];

    if T::force_md5_keyhash() || key_cdr.len() > 16 {
        let mut md5st = ddsrt_md5_state_t::default();
        let md5set = &mut md5st as *mut ddsrt_md5_state_s;
        unsafe {
            ddsrt_md5_init(md5set);
            ddsrt_md5_append(md5set, key_cdr.as_ptr(), key_cdr.len() as u32);
            ddsrt_md5_finish(md5set, cdr_key.as_mut_ptr());
        }
    } else {
        for (i, data) in key_cdr.iter().enumerate() {
            cdr_key[i] = *data;
        }
    }
    serdata.key_hash = KeyHash::CdrKey(cdr_key)
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_from_keyhash<T>(
    sertype: *const ddsi_sertype,
    keyhash: *const ddsi_keyhash,
) -> *mut ddsi_serdata
where
    T: TopicType,
{
    let keyhash = (*keyhash).value;
    //println!("serdata_from_keyhash");

    if T::force_md5_keyhash() {
        // this means keyhas fits in 16 bytes
        std::ptr::null_mut()
    } else {
        let mut serdata = SerData::<T>::new(sertype, ddsi_serdata_kind_SDK_KEY);
        serdata.sample = SampleData::SDKKey;

        let mut key_hash_buffer = [0u8; 20];
        let key_hash = &mut key_hash_buffer[4..];

        for (i, b) in keyhash.iter().enumerate() {
            key_hash[i] = *b;
        }

        serdata.key_hash = KeyHash::CdrKey(key_hash_buffer);

        let ptr = Box::into_raw(serdata);
        // only we know this ddsi_serdata is really of type SerData
        ptr as *mut ddsi_serdata
    }
}

#[allow(dead_code)]
#[allow(non_upper_case_globals)]
unsafe extern "C" fn serdata_from_sample<T>(
    sertype: *const ddsi_sertype,
    kind: u32,
    sample: *const c_void,
) -> *mut ddsi_serdata
where
    T: TopicType,
{
    //println!("Serdata from sample {:?}", sample);
    let mut serdata = SerData::<T>::new(sertype, kind);
    let sample = sample as *const Sample<T>;
    let sample = &*sample;

    match kind {
        #[allow(non_upper_case_globals)]
        ddsi_serdata_kind_SDK_DATA => {
            let sample = sample.get().unwrap();
            serdata.serdata.hash = sample.hash((*sertype).serdata_basehash);
            serdata.sample = SampleData::SDKData(sample);
        }
        ddsi_serdata_kind_SDK_KEY => {
            panic!("Don't know how to create serdata from sample for SDK_KEY");
        }
        _ => panic!("Unexpected kind"),
    }

    let ptr = Box::into_raw(serdata);
    // only we know this ddsi_serdata is really of type SerData
    ptr as *mut ddsi_serdata
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_from_iov<T>(
    sertype: *const ddsi_sertype,
    kind: u32,
    niov: size_t,
    iov: *const iovec,
    size: size_t,
) -> *mut ddsi_serdata
where
    T: DeserializeOwned + TopicType,
{
    let size = size as usize;
    let niov = niov as usize;
    //println!("serdata_from_iov");

    let mut serdata = SerData::<T>::new(sertype, kind);

    let iovs = std::slice::from_raw_parts(iov as *const cyclonedds_sys::iovec, niov);

    let iov_slices: Vec<&[u8]> = iovs
        .iter()
        .map(|iov| {
            let iov = iov;

            std::slice::from_raw_parts(iov.iov_base as *const u8, iov.iov_len as usize)
        })
        .collect();

    // make a reader out of the sg_list
    let reader = SGReader::new(&iov_slices);

    if let Ok(decoded) = cdr::deserialize_from::<_, T, _>(reader, Bounded(size as u64)) {
        if T::has_key() {
            // compute the 16byte key hash
            let key_cdr = decoded.key_cdr();
            // skip the four byte header
            let key_cdr = &key_cdr[4..];
            compute_key_hash(key_cdr, &mut serdata);
        }
        serdata.serdata.hash = decoded.hash((*sertype).serdata_basehash);
        let sample = std::sync::Arc::new(decoded);
        //store the deserialized sample in the serdata. We don't need to deserialize again
        serdata.sample = SampleData::SDKData(sample);
    } else {
        //println!("Deserialization error!");
        return std::ptr::null_mut();
    }

    // convert into raw pointer and forget about it as ownership is passed into cyclonedds
    let ptr = Box::into_raw(serdata);
    // only we know this ddsi_serdata is really of type SerData
    ptr as *mut ddsi_serdata
}

#[allow(dead_code)]
unsafe extern "C" fn free_serdata<T>(serdata: *mut ddsi_serdata) {
    //println!("free_serdata");
    // the pointer is really a *mut SerData
    let ptr = serdata as *mut SerData<T>;

    let serdata = &mut *ptr;

    #[cfg(feature = "shm")]
    if !serdata.serdata.iox_subscriber.is_null() {
        let iox_subscriber: *mut iox_sub_t = serdata.serdata.iox_subscriber as *mut iox_sub_t;
        let chunk = &mut serdata.serdata.iox_chunk;
        let chunk = chunk as *mut *mut c_void;
        //println!("Free iox chunk");
        free_iox_chunk(iox_subscriber, chunk);
    }

    let _data = Box::from_raw(ptr);
    // _data goes out of scope and frees the SerData. Nothing more to do here.
}

#[allow(dead_code)]
unsafe extern "C" fn get_size<T>(serdata: *const ddsi_serdata) -> u32
where
    T: Serialize + TopicType,
{
    let serdata = SerData::<T>::mut_ref_from_serdata(serdata);
    let size = match &serdata.sample {
        SampleData::Uninitialized => 0,
        SampleData::SDKKey => serdata.key_hash.key_length() as u32,
        // This function asks for the serialized size so we do this even for SHM Data
        SampleData::SDKData(sample) => {
            serdata.serialized_size = Some((cdr::calc_serialized_size::<T>(sample.deref())) as u32);
            *serdata.serialized_size.as_ref().unwrap()
        }
        SampleData::SHMData(_sample) => {
            // we refuse to serialize SHM data so return 0
            0
            /*
            serdata.serialized_size = Some((cdr::calc_serialized_size::<T>(sample.as_ref())) as u32);
            *serdata.serialized_size.as_ref().unwrap()
            */
        }
    };
    size
}

#[allow(dead_code)]
unsafe extern "C" fn eqkey<T>(
    serdata_a: *const ddsi_serdata,
    serdata_b: *const ddsi_serdata,
) -> bool {
    let a = SerData::<T>::mut_ref_from_serdata(serdata_a);
    let b = SerData::<T>::mut_ref_from_serdata(serdata_b);
    a.key_hash == b.key_hash
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_to_ser<T>(
    serdata: *const ddsi_serdata,
    size: size_t,
    offset: size_t,
    buf: *mut c_void,
) where
    T: Serialize + TopicType,
{
    //println!("serdata_to_ser");
    let serdata = SerData::<T>::const_ref_from_serdata(serdata);
    let buf = buf as *mut u8;
    let buf = buf.add(offset as usize);

    if size == 0 {
        return;
    }

    match &serdata.sample {
        SampleData::Uninitialized => {
            panic!("Attempt to serialize uninitialized serdata")
        }
        SampleData::SDKKey => match &serdata.key_hash {
            KeyHash::None => {}
            KeyHash::CdrKey(k) => std::ptr::copy_nonoverlapping(k.as_ptr(), buf, size as usize),
            KeyHash::RawKey(k) => std::ptr::copy_nonoverlapping(k.as_ptr(), buf, size as usize),
        },
        // We may serialize both SDK data as well as SHM Data
        SampleData::SDKData(serdata) => {
            let buf_slice = std::slice::from_raw_parts_mut(buf, size as usize);
            if let Err(e) =
                cdr::serialize_into::<_, T, _, CdrBe>(buf_slice, serdata.deref(), Bounded(size))
            {
                panic!("Unable to serialize type {:?} due to {}", T::typename(), e);
            }
        }
        SampleData::SHMData(serdata) => {
            let buf_slice = std::slice::from_raw_parts_mut(buf, size as usize);
            if let Err(e) =
                cdr::serialize_into::<_, T, _, CdrBe>(buf_slice, serdata.as_ref(), Bounded(size))
            {
                panic!("Unable to serialize type {:?} due to {}", T::typename(), e);
            }
        }
    }
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_to_ser_ref<T>(
    serdata: *const ddsi_serdata,
    offset: size_t,
    size: size_t,
    iov: *mut iovec,
) -> *mut ddsi_serdata
where
    T: Serialize + TopicType,
{
    //println!("serdata_to_ser_ref");
    let serdata = SerData::<T>::mut_ref_from_serdata(serdata);
    let iov = &mut *iov;

    match &serdata.sample {
        SampleData::Uninitialized => panic!("Attempt to serialize uninitialized Sample"),
        SampleData::SDKKey => {
            let (p, len) = match &serdata.key_hash {
                KeyHash::None => (std::ptr::null(), 0),
                KeyHash::CdrKey(k) => (k.as_ptr(), k.len()),
                KeyHash::RawKey(k) => (k.as_ptr(), k.len()),
            };

            iov.iov_base = p as *mut c_void;
            iov.iov_len = len as size_t;
        }
        SampleData::SDKData(sample) => {
            if serdata.cdr.is_none() {
                serdata.cdr = serialize_type::<T>(sample, serdata.serialized_size).ok();
            }
            if let Some(cdr) = &serdata.cdr {
                let offset = offset as usize;
                let mut last = offset + size as usize;
                if last > cdr.len() - 1 {
                    last = cdr.len() - 1;
                }
                let cdr = &cdr[offset..last];
                // cdds rounds up the length into multiple of 4. We mirror that by allocating extra in the
                // ``serialize_type`` function.
                iov.iov_base = cdr.as_ptr() as *mut c_void;
                iov.iov_len = size; //cdr.len() as size_t;
            } else {
                println!("Serialization error!");
                return std::ptr::null_mut();
            }
        }

        SampleData::SHMData(sample) => {
            if serdata.cdr.is_none() {
                serdata.cdr = serialize_type::<T>(sample.as_ref(), serdata.serialized_size).ok();
            }
            if let Some(cdr) = &serdata.cdr {
                let offset = offset as usize;
                let last = offset + size as usize;
                let cdr = &cdr[offset..last];
                iov.iov_base = cdr.as_ptr() as *mut c_void;
                iov.iov_len = cdr.len() as size_t;
            } else {
                println!("Serialization error (SHM)!");
                return std::ptr::null_mut();
            }
        }
    }
    ddsi_serdata_addref(&serdata.serdata)
}

fn serialize_type<T: Serialize>(sample: &T, maybe_size: Option<u32>) -> Result<Vec<u8>, ()> {
    if let Some(size) = maybe_size {
        // Round up allocation to multiple of four
        let size = (size + 3) & !3u32;
        let mut buffer = Vec::<u8>::with_capacity(size as usize);
        if let Ok(()) = cdr::serialize_into::<_, T, _, CdrBe>(&mut buffer, sample, Infinite) {
            Ok(buffer)
        } else {
            Err(())
        }
    } else if let Ok(data) = cdr::serialize::<T, _, CdrBe>(sample, Infinite) {
        Ok(data)
    } else {
        Err(())
    }
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_to_ser_unref<T>(serdata: *mut ddsi_serdata, _iov: *const iovec) {
    //println!("serdata_to_ser_unref");
    let serdata = SerData::<T>::mut_ref_from_serdata(serdata);
    ddsi_serdata_removeref(&mut serdata.serdata)
}

fn deserialize_type<T>(data: &[u8]) -> Result<Arc<T>, ()>
where
    T: DeserializeOwned,
{
    cdr::deserialize::<Box<T>>(data)
        .map(Arc::from)
        .map_err(|_e| ())
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_to_sample<T>(
    serdata_ptr: *const ddsi_serdata,
    sample: *mut c_void,
    _bufptr: *mut *mut c_void,
    _buflim: *mut c_void,
) -> bool
where
    T: DeserializeOwned + TopicType,
{
    //println!(
    //    "serdata to sample serdata:{:?} sample:{:?} bufptr:{:?} buflim:{:?}",
    //    serdata, sample, _bufptr, _buflim
    //);
    let mut serdata = SerData::<T>::mut_ref_from_serdata(serdata_ptr);
    let mut s = Box::<Sample<T>>::from_raw(sample as *mut Sample<T>);
    assert!(!sample.is_null());

    #[cfg(feature = "shm")]
    let ret = if !serdata.serdata.iox_chunk.is_null() {
        // We got data from Iceoryx, deal with it
        let hdr = iceoryx_header_from_chunk(serdata.serdata.iox_chunk);
        if (*hdr).shm_data_state == iox_shm_data_state_t_IOX_CHUNK_CONTAINS_SERIALIZED_DATA {
            // we have to deserialize the data now
            let reader = std::slice::from_raw_parts(
                serdata.serdata.iox_chunk as *const u8,
                (*hdr).data_size as usize,
            );
            if serdata.serdata.kind == ddsi_serdata_kind_SDK_KEY {
                compute_key_hash(reader, serdata);
                serdata.sample = SampleData::SDKKey;
                Ok(())
            } else if let Ok(decoded) = deserialize_type::<T>(reader) {
                if T::has_key() {
                    // compute the 16byte key hash
                    let key_cdr = decoded.key_cdr();
                    // skip the four byte header
                    let key_cdr = &key_cdr[4..];
                    compute_key_hash(key_cdr, serdata);
                }
                //let sample = std::sync::Arc::new(decoded);
                //store the deserialized sample in the serdata. We don't need to deserialize again
                s.set(decoded.clone());
                serdata.sample = SampleData::SDKData(decoded);

                Ok(())
            } else {
                println!("Deserialization error!");
                Err(())
            }
        } else {
            // Not serialized data, we make a sample out of the data and store it in our sample
            assert_eq!((*hdr).data_size as usize, std::mem::size_of::<T>());
            if std::mem::size_of::<T>() == (*hdr).data_size as usize {
                // Pay Attention here
                //
                //
                let p: *mut T = serdata.serdata.iox_chunk as *mut T;
                serdata.sample = SampleData::SHMData(NonNull::new_unchecked(p));
                Ok(())
            } else {
                Err(())
            }
        }
    } else {
        Ok(())
    };
    #[cfg(not(feature = "shm"))]
    let ret: Result<(), ()> = Ok(());

    let ret = if let Ok(()) = ret {
        match &serdata.sample {
            SampleData::Uninitialized => true,
            SampleData::SDKKey => true,
            SampleData::SDKData(_data) => {
                s.set_serdata(serdata_ptr as *mut ddsi_serdata);
                //s.set(data.clone());
                false
            }
            SampleData::SHMData(_data) => {
                s.set_serdata(serdata_ptr as *mut ddsi_serdata);
                //s.set_loaned(data.clone());
                false
            }
        }
    } else {
        true
    };

    // leak the sample intentionally so it doesn't get deallocated here
    let _intentional_leak = Box::into_raw(s);
    ret
}

#[allow(dead_code)]
unsafe extern "C" fn serdata_to_untyped<T>(serdata: *const ddsi_serdata) -> *mut ddsi_serdata {
    //println!("serdata_to_untyped {:?}", serdata);
    let serdata = SerData::<T>::mut_ref_from_serdata(serdata);

    //if let SampleData::<T>::SDKData(_d) = &serdata.sample {
    let mut untyped_serdata = SerData::<T>::new(serdata.serdata.type_, ddsi_serdata_kind_SDK_KEY);
    // untype it
    untyped_serdata.serdata.type_ = std::ptr::null_mut();
    untyped_serdata.sample = SampleData::SDKKey;

    //copy the hashes
    untyped_serdata.key_hash = serdata.key_hash.clone();
    untyped_serdata.serdata.hash = serdata.serdata.hash;

    let ptr = Box::into_raw(untyped_serdata);

    ptr as *mut ddsi_serdata
    //} else {
    //    println!("Error: Cannot convert from untyped to untyped");
    //    std::ptr::null_mut()
    //}
}

#[allow(dead_code)]
unsafe extern "C" fn untyped_to_sample<T>(
    _sertype: *const ddsi_sertype,
    _serdata: *const ddsi_serdata,
    sample: *mut c_void,
    _buf: *mut *mut c_void,
    _buflim: *mut c_void,
) -> bool
where
    T: TopicType,
{
    //println!("untyped to sample!");
    if !sample.is_null() {
        let mut sample = Box::<Sample<T>>::from_raw(sample as *mut Sample<T>);
        // hmm. We don't store serialized data in serdata. I'm not really sure how
        // to implement this. For now, invalidate the sample.
        sample.clear();
        // leak this as we don't want to deallocate it.
        let _leaked = Box::<Sample<T>>::into_raw(sample);
        true
    } else {
        false
    }
}

#[allow(dead_code)]
unsafe extern "C" fn get_keyhash<T>(
    serdata: *const ddsi_serdata,
    keyhash: *mut ddsi_keyhash,
    _force_md5: bool,
) {
    let serdata = SerData::<T>::mut_ref_from_serdata(serdata);
    let keyhash = &mut *keyhash;

    let src = match &serdata.key_hash {
        KeyHash::None => &[],
        KeyHash::CdrKey(k) => &k[4..],
        KeyHash::RawKey(k) => &k[..],
    };

    //let source_key_hash = &serdata.key_hash[4..];
    for (i, b) in src.iter().enumerate() {
        keyhash.value[i] = *b;
    }
}

#[allow(dead_code)]
unsafe extern "C" fn print<T>(
    _sertype: *const ddsi_sertype,
    _serdata: *const ddsi_serdata,
    _buf: *mut std::os::raw::c_char,
    _bufsize: size_t,
) -> size_t {
    0
}

fn create_sertype_ops<T>() -> Box<ddsi_sertype_ops>
where
    T: TopicType,
{
    Box::new(ddsi_sertype_ops {
        version: Some(ddsi_sertype_v0),
        arg: std::ptr::null_mut(),
        free: Some(free_sertype::<T>),
        zero_samples: Some(zero_samples::<T>),
        realloc_samples: Some(realloc_samples::<T>),
        free_samples: Some(free_samples::<T>),
        equal: Some(equal::<T>),
        hash: Some(hash::<T>),
        ..Default::default()
    })
}

#[cfg(feature = "shm")]
#[allow(dead_code)]
unsafe extern "C" fn get_sample_size(serdata: *const ddsi_serdata) -> u32 {
    let serdata = *serdata;
    (*serdata.type_).iox_size
}

#[cfg(feature = "shm")]
#[allow(dead_code)]
unsafe extern "C" fn from_iox_buffer<T>(
    sertype: *const ddsi_sertype,
    kind: ddsi_serdata_kind,
    /*_deserialize_hint : bool,*/
    sub: *mut ::std::os::raw::c_void,
    buffer: *mut ::std::os::raw::c_void,
) -> *mut ddsi_serdata {
    //println!("from_iox_buffer");

    if sertype.is_null() {
        return std::ptr::null::<ddsi_serdata>() as *mut ddsi_serdata;
    }

    let mut d = SerData::<T>::new(sertype, kind);

    // from loaned sample, just take the pointer
    if sub.is_null() {
        d.serdata.iox_chunk = buffer;
    } else {
        //println!("from_iox_buffer: take pointer {:?}from iox", buffer);
        // from iox buffer
        d.serdata.iox_chunk = buffer;
        d.serdata.iox_subscriber = sub;
        let hdr = iceoryx_header_from_chunk(buffer);
        // Copy the key hash (TODO: Check this)
        copy_raw_key_hash(&(*hdr).keyhash.value, &mut d);
    }

    // we don't deserialize right away
    d.sample = SampleData::SHMData(NonNull::new_unchecked(buffer as *mut T));

    let ptr = Box::into_raw(d);
    // only we know this ddsi_serdata is really of type SerData
    ptr as *mut ddsi_serdata
}

fn create_serdata_ops<T>() -> Box<ddsi_serdata_ops>
where
    T: DeserializeOwned + TopicType + Serialize,
{
    Box::new(ddsi_serdata_ops {
        eqkey: Some(eqkey::<T>),
        get_size: Some(get_size::<T>),
        from_ser: Some(serdata_from_fragchain::<T>),
        from_ser_iov: Some(serdata_from_iov::<T>),
        from_keyhash: Some(serdata_from_keyhash::<T>),
        from_sample: Some(serdata_from_sample::<T>),
        to_ser: Some(serdata_to_ser::<T>),
        to_ser_ref: Some(serdata_to_ser_ref::<T>),
        to_ser_unref: Some(serdata_to_ser_unref::<T>),
        to_sample: Some(serdata_to_sample::<T>),
        to_untyped: Some(serdata_to_untyped::<T>),
        untyped_to_sample: Some(untyped_to_sample::<T>),
        free: Some(free_serdata::<T>),
        print: Some(print::<T>),
        get_keyhash: Some(get_keyhash::<T>),
        #[cfg(feature = "shm")]
        get_sample_size: Some(get_sample_size),
        #[cfg(feature = "shm")]
        from_iox_buffer: Some(from_iox_buffer::<T>),
        ..Default::default()
    })
}

// not sure what this needs to do. The C++ implementation at
// https://github.com/eclipse-cyclonedds/cyclonedds-cxx/blob/templated-streaming/src/ddscxx/include/org/eclipse/cyclonedds/topic/datatopic.hpp
// just returns 0
// Update! : Now I understand this after debugging crashes when stress testing
// with a large number of types being published. This hash is used as the hash
// lookup in hopscotch.c.
// /*
//  * The hopscotch hash table is dependent on a proper functioning hash.
//  * If the hash function generates a lot of hash collisions, then it will
//  * not be able to handle that by design.
//  * It is capable of handling some collisions, but not more than 32 per
//  * bucket (less, when other hash values are clustered around the
//  * collision value).
//  * When proper distributed hash values are generated, then hopscotch
//  * works nice and quickly.
//  */
unsafe extern "C" fn hash<T: TopicType>(tp: *const ddsi_sertype) -> u32 {
    if let Some(ser_type) = SerType::<T>::try_from_sertype(tp) {
        let type_name = CStr::from_ptr(ser_type.sertype.type_name);
        let type_name_bytes = type_name.to_bytes();
        let type_size = core::mem::size_of::<T>().to_ne_bytes();
        let sg_list = [type_name_bytes, &type_size];
        let mut sg_buffer = SGReader::new(&sg_list);

        let hash = murmur3_32(&mut sg_buffer, 0);

        let _intentional_leak = SerType::<T>::into_sertype(ser_type);
        hash.unwrap_or(0)
    } else {
        0
    }
}

unsafe extern "C" fn equal<T>(acmn: *const ddsi_sertype, bcmn: *const ddsi_sertype) -> bool {
    let acmn = CStr::from_ptr((*acmn).type_name as *mut std::os::raw::c_char);
    let bcmn = CStr::from_ptr((*bcmn).type_name as *mut std::os::raw::c_char);
    acmn == bcmn
}

#[derive(Clone)]
enum SampleData<T> {
    Uninitialized,
    SDKKey,
    SDKData(std::sync::Arc<T>),
    SHMData(NonNull<T>),
}

impl<T> Default for SampleData<T> {
    fn default() -> Self {
        Self::Uninitialized
    }
}

#[derive(PartialEq, Clone)]
enum KeyHash {
    None,
    CdrKey([u8; 20]),
    RawKey([u8; 16]),
}

impl Default for KeyHash {
    fn default() -> Self {
        Self::None
    }
}

impl KeyHash {
    fn get_key_hash(&self) -> &[u8] {
        match self {
            KeyHash::None => &[],
            KeyHash::CdrKey(cdr_key_hash) => cdr_key_hash,
            KeyHash::RawKey(raw_key_hash) => raw_key_hash,
        }
    }
    fn key_length(&self) -> usize {
        match self {
            KeyHash::CdrKey(k) => k.len(),
            KeyHash::RawKey(k) => k.len(),
            _ => 0,
        }
    }
}

/// A representation for the serialized data.
#[repr(C)]
pub(crate) struct SerData<T> {
    serdata: ddsi_serdata,
    sample: SampleData<T>,
    //data in CDR format. This is put into an option as we only create
    //the serialized version when we need it
    cdr: Option<Vec<u8>>,
    //key_hash: ddsi_keyhash,
    // include 4 bytes of CDR encapsulation header
    //key_hash: [u8; 20],
    key_hash: KeyHash,
    // We store the serialized size here if available
    serialized_size: Option<u32>,
}

impl<'a, T> SerData<T> {
    fn new(sertype: *const ddsi_sertype, kind: u32) -> Box<SerData<T>> {
        Box::<SerData<T>>::new(SerData {
            serdata: {
                let mut data = std::mem::MaybeUninit::uninit();
                unsafe {
                    ddsi_serdata_init(data.as_mut_ptr(), sertype, kind);
                    data.assume_init()
                }
            },
            sample: SampleData::default(),
            cdr: None,
            key_hash: KeyHash::default(),
            serialized_size: None,
        })
    }

    fn const_ref_from_serdata(serdata: *const ddsi_serdata) -> &'a Self {
        let ptr = serdata as *const SerData<T>;
        unsafe { &*ptr }
    }

    fn mut_ref_from_serdata(serdata: *const ddsi_serdata) -> &'a mut Self {
        let ptr = serdata as *mut SerData<T>;
        unsafe { &mut *ptr }
    }
}

impl<T> Clone for SerData<T> {
    fn clone(&self) -> Self {
        Self {
            serdata: {
                let mut newdata = self.serdata;
                unsafe { ddsi_serdata_addref(&mut newdata) };
                newdata
            },
            sample: match &self.sample {
                SampleData::Uninitialized => SampleData::Uninitialized,
                SampleData::SDKKey => SampleData::SDKKey,
                SampleData::SDKData(d) => SampleData::SDKData(d.clone()),
                SampleData::SHMData(d) => SampleData::SHMData(*d),
            },
            cdr: self.cdr.clone(),
            key_hash: self.key_hash.clone(),
            serialized_size: self.serialized_size,
        }
    }
}

/*  These functions are created from the macros in
    https://github.com/eclipse-cyclonedds/cyclonedds/blob/f879dc0ef56eb00857c0cbb66ee87c577ff527e8/src/core/ddsi/include/dds/ddsi/q_radmin.h#L108
    Bad things will happen if these macros change.
    Some discussions here: https://github.com/eclipse-cyclonedds/cyclonedds/issues/830
*/
fn nn_rdata_payload_offset(rdata: *const nn_rdata) -> usize {
    unsafe { (*rdata).payload_zoff as usize }
}

fn nn_rmsg_payload(rmsg: *const nn_rmsg) -> *const u8 {
    unsafe { rmsg.add(1) as *const u8 }
}

fn nn_rmsg_payload_offset(rmsg: *const nn_rmsg, offset: usize) -> *const u8 {
    unsafe { nn_rmsg_payload(rmsg).add(offset) }
}

/// A reader for a list of scatter gather buffers
struct SGReader<'a> {
    sc_list: Option<&'a [&'a [u8]]>,
    //the current slice that is used
    slice_cursor: usize,
    //the current offset within the slice
    slice_offset: usize,
}

impl<'a> SGReader<'a> {
    pub fn new(sc_list: &'a [&'a [u8]]) -> Self {
        SGReader {
            sc_list: Some(sc_list),
            slice_cursor: 0,
            slice_offset: 0,
        }
    }
}

impl<'a> Read for SGReader<'a> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let read_buf_len = buf.len();
        if self.sc_list.is_some() {
            let source_slice = self.sc_list.as_ref().unwrap()[self.slice_cursor];
            let num_slices = self.sc_list.as_ref().unwrap().len();
            let source_slice_rem = source_slice.len() - self.slice_offset;
            let source_slice = &source_slice[self.slice_offset..];

            let copy_length = std::cmp::min(source_slice_rem, read_buf_len);

            //copy the bytes, lengths have to be the same
            buf[..copy_length].copy_from_slice(&source_slice[..copy_length]);

            if copy_length == source_slice_rem {
                // we have completed this slice. move to the next
                self.slice_cursor += 1;
                self.slice_offset = 0;

                if self.slice_cursor >= num_slices {
                    //no more slices, invalidate the sc_list
                    let _ = self.sc_list.take();
                }
            } else {
                // we have not completed the current slice, just bump up the slice offset
                self.slice_offset += copy_length;
            }

            Ok(copy_length)
        } else {
            // No more data
            Ok(0)
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{DdsListener, DdsParticipant, DdsQos, DdsTopic};
    use cdds_derive::Topic;
    use serde_derive::{Deserialize, Serialize};
    use std::ffi::CString;

    #[test]
    fn scatter_gather() {
        let a = vec![1, 2, 3, 4, 5, 6];
        let b = vec![7, 8, 9, 10, 11];
        let c = vec![12, 13, 14, 15];
        let d = vec![16, 17, 18, 19, 20, 21];

        let sla = unsafe { std::slice::from_raw_parts(a.as_ptr(), a.len()) };
        let slb = unsafe { std::slice::from_raw_parts(b.as_ptr(), b.len()) };
        let slc = unsafe { std::slice::from_raw_parts(c.as_ptr(), c.len()) };
        let sld = unsafe { std::slice::from_raw_parts(d.as_ptr(), d.len()) };

        let sc_list = vec![sla, slb, slc, sld];

        let mut reader = SGReader::new(&sc_list);

        let mut buf = vec![0, 0, 0, 0, 0];
        if let Ok(n) = reader.read(&mut buf) {
            assert_eq!(&buf[..n], vec![1, 2, 3, 4, 5]);
        } else {
            panic!("should not panic");
        }
        if let Ok(n) = reader.read(&mut buf) {
            assert_eq!(&buf[..n], vec![6]);
        } else {
            panic!("should not panic");
        }
    }

    #[test]
    fn keyhash_basic() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            id: i32,
            x: u32,
            y: u32,
        }
        let foo = Foo {
            id: 0x12345678,
            x: 10,
            y: 20,
        };
        let key_cdr = foo.key_cdr();
        assert_eq!(key_cdr, vec![0, 0, 0, 0, 0x12u8, 0x34u8, 0x56u8, 0x78u8]);
    }
    #[test]
    fn keyhash_simple() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            id: i32,
            x: u32,
            #[topic_key]
            s: String,
            y: u32,
        }
        let foo = Foo {
            id: 0x12345678,
            x: 10,
            s: String::from("boo"),
            y: 20,
        };
        let key_cdr = foo.key_cdr();
        assert_eq!(
            key_cdr,
            vec![0, 0, 0, 0, 18, 52, 86, 120, 0, 0, 0, 4, 98, 111, 111, 0]
        );
    }

    #[test]
    fn keyhash_nested() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct NestedFoo {
            name: String,
            val: u64,
            #[topic_key]
            instance: u32,
        }

        assert_eq!(
            NestedFoo::typename(),
            CString::new("serdes::test::keyhash_nested::NestedFoo").unwrap()
        );

        impl NestedFoo {
            fn new() -> Self {
                Self {
                    name: "my name".to_owned(),
                    val: 42,
                    instance: 25,
                }
            }
        }

        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            id: i32,
            x: u32,
            #[topic_key]
            s: String,
            y: u32,
            #[topic_key]
            inner: NestedFoo,
        }
        let foo = Foo {
            id: 0x12345678,
            x: 10,
            s: String::from("boo"),
            y: 20,
            inner: NestedFoo::new(),
        };
        let key_cdr = foo.key_cdr();
        assert_eq!(
            key_cdr,
            vec![0, 0, 0, 0, 18, 52, 86, 120, 0, 0, 0, 4, 98, 111, 111, 0, 0, 0, 0, 25]
        );
    }

    #[test]
    fn primitive_array_as_key() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            a: [u8; 8],
            b: u32,
            c: String,
        }

        let foo = Foo {
            a: [0, 0, 0, 0, 0, 0, 0, 0],
            b: 42,
            c: "foo".to_owned(),
        };

        let key_cdr = foo.key_cdr();
        assert_eq!(key_cdr, vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(false, Foo::force_md5_keyhash());
    }

    #[test]
    fn primitive_array_and_string_as_key() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            a: [u8; 8],
            b: u32,
            #[topic_key]
            c: String,
        }

        let foo = Foo {
            a: [0, 0, 0, 0, 0, 0, 0, 0],
            b: 42,
            c: "foo".to_owned(),
        };

        let key_cdr = foo.key_cdr();
        assert_eq!(
            key_cdr,
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 102, 111, 111, 0]
        );
        assert_eq!(true, Foo::force_md5_keyhash());
    }

    #[test]
    fn basic() {
        #[derive(Serialize, Deserialize, Topic, Default)]
        struct NestedFoo {
            name: String,
            val: u64,
            #[topic_key]
            instance: u32,
        }

        impl NestedFoo {
            fn new() -> Self {
                Self {
                    name: "my name".to_owned(),
                    val: 42,
                    instance: 25,
                }
            }
        }

        #[derive(Serialize, Deserialize, Topic, Default)]
        struct Foo {
            #[topic_key]
            id: i32,
            x: u32,
            #[topic_key]
            s: String,
            y: u32,
            #[topic_key]
            inner: NestedFoo,
        }
        let _foo = Foo {
            id: 0x12345678,
            x: 10,
            s: String::from("boo"),
            y: 20,
            inner: NestedFoo::new(),
        };
        let t = SerType::<Foo>::new();
        let mut t = SerType::into_sertype(t);
        let tt = &mut t as *mut *mut ddsi_sertype;
        unsafe {
            let p = dds_create_participant(0, std::ptr::null_mut(), std::ptr::null_mut());
            let topic_name = CString::new("topic_name").unwrap();
            let topic = dds_create_topic_sertype(
                p,
                topic_name.as_ptr(),
                tt,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            );

            dds_delete(topic);
        }
    }
}
