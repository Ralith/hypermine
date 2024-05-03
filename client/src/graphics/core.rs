use std::ffi::CStr;
use std::os::raw::c_char;
use std::os::raw::c_void;
use std::ptr;
use std::slice;

use ash::ext::debug_utils;
use ash::{vk, Entry, Instance};
use tracing::{debug, error, info, trace, warn};

use common::defer;

/// The most fundamental components of a Vulkan setup
pub struct Core {
    /// Handle to the Vulkan dynamic library itself, used to bootstrap
    pub entry: Entry,
    /// The Vulkan instance, containing fundamental device-independent functions
    pub instance: Instance,

    /// Diagnostic infrastructure, configured if the environment supports them. Typically present
    /// when the Vulkan validation layers are enabled or a graphics debugger is in use and absent
    /// otherwise.
    pub debug_utils: Option<debug_utils::Instance>,
    messenger: vk::DebugUtilsMessengerEXT,
}

impl Drop for Core {
    fn drop(&mut self) {
        unsafe {
            if let Some(ref utils) = self.debug_utils {
                utils.destroy_debug_utils_messenger(self.messenger, None);
            }
            self.instance.destroy_instance(None);
        }
    }
}

impl Core {
    pub fn new(exts: &[*const c_char]) -> Self {
        unsafe {
            let entry = Entry::load().unwrap();

            let supported_exts = entry.enumerate_instance_extension_properties(None).unwrap();
            let has_debug = supported_exts
                .iter()
                .any(|x| CStr::from_ptr(x.extension_name.as_ptr()) == debug_utils::NAME);

            let mut exts = exts.to_vec();
            if has_debug {
                exts.push(debug_utils::NAME.as_ptr());
            } else {
                info!("vulkan debugging unavailable");
            }

            let instance_layers = entry.enumerate_instance_layer_properties().unwrap();
            tracing::info!(
                "Vulkan instance layers: {:?}",
                instance_layers
                    .iter()
                    .map(|layer| CStr::from_ptr(layer.layer_name.as_ptr()).to_str().unwrap())
                    .collect::<Vec<_>>()
            );

            let name = cstr!("hypermine");

            let app_info = vk::ApplicationInfo::default()
                .application_name(name)
                .application_version(0)
                .engine_name(name)
                .engine_version(0)
                .api_version(vk::make_api_version(0, 1, 1, 0));
            let mut instance_info = vk::InstanceCreateInfo::default()
                .application_info(&app_info)
                .enabled_extension_names(&exts);

            let mut debug_utils_messenger_info = vk::DebugUtilsMessengerCreateInfoEXT::default()
                .message_severity(
                    vk::DebugUtilsMessageSeverityFlagsEXT::ERROR
                        | vk::DebugUtilsMessageSeverityFlagsEXT::WARNING
                        | vk::DebugUtilsMessageSeverityFlagsEXT::INFO
                        | vk::DebugUtilsMessageSeverityFlagsEXT::VERBOSE,
                )
                .message_type(
                    vk::DebugUtilsMessageTypeFlagsEXT::GENERAL
                        | vk::DebugUtilsMessageTypeFlagsEXT::VALIDATION
                        | vk::DebugUtilsMessageTypeFlagsEXT::PERFORMANCE,
                )
                .pfn_user_callback(Some(messenger_callback))
                .user_data(ptr::null_mut());
            if has_debug {
                instance_info = instance_info.push_next(&mut debug_utils_messenger_info);
            }

            let instance = entry.create_instance(&instance_info, None).unwrap();
            // Guards ensure we clean up gracefully if something panics
            let instance_guard = defer(|| instance.destroy_instance(None));
            let debug_utils;
            let messenger;
            if has_debug {
                // Configure Vulkan diagnostic message logging
                let utils = debug_utils::Instance::new(&entry, &instance);
                messenger = utils
                    .create_debug_utils_messenger(&debug_utils_messenger_info, None)
                    .unwrap();
                debug_utils = Some(utils);
            } else {
                debug_utils = None;
                messenger = vk::DebugUtilsMessengerEXT::null();
            }

            // Setup successful, don't destroy things.
            instance_guard.cancel();
            Self {
                entry,
                instance,
                debug_utils,
                messenger,
            }
        }
    }
}

/// Callback invoked by Vulkan for diagnostic messages
///
/// We forward these to our `tracing` logging infrastructure.
unsafe extern "system" fn messenger_callback(
    message_severity: vk::DebugUtilsMessageSeverityFlagsEXT,
    _message_types: vk::DebugUtilsMessageTypeFlagsEXT,
    p_data: *const vk::DebugUtilsMessengerCallbackDataEXT,
    _p_user_data: *mut c_void,
) -> vk::Bool32 {
    unsafe fn fmt_labels(ptr: *const vk::DebugUtilsLabelEXT, count: u32) -> String {
        if count == 0 {
            // We need to handle a count of 0 separately because ptr may be
            // null, resulting in undefined behavior if used with
            // slice::from_raw_parts.
            return String::new();
        }
        slice::from_raw_parts(ptr, count as usize)
            .iter()
            .map(|label| {
                CStr::from_ptr(label.p_label_name)
                    .to_string_lossy()
                    .into_owned()
            })
            .collect::<Vec<_>>()
            .join(", ")
    }

    let data = &*p_data;
    let msg_id = if data.p_message_id_name.is_null() {
        "".into()
    } else {
        CStr::from_ptr(data.p_message_id_name).to_string_lossy()
    };
    let msg = CStr::from_ptr(data.p_message).to_string_lossy();
    let queue_labels = fmt_labels(data.p_queue_labels, data.queue_label_count);
    let cmd_labels = fmt_labels(data.p_cmd_buf_labels, data.cmd_buf_label_count);
    let objects = slice::from_raw_parts(data.p_objects, data.object_count as usize)
        .iter()
        .map(|obj| {
            if obj.p_object_name.is_null() {
                format!("{:?} {:x}", obj.object_type, obj.object_handle)
            } else {
                format!("{:?} {:x} {}", obj.object_type, obj.object_handle, msg_id)
            }
        })
        .collect::<Vec<_>>()
        .join(", ");
    if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::ERROR {
        error!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::WARNING {
        warn! (target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::INFO {
        debug!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else {
        trace!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    }
    vk::FALSE
}
