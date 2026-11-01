// Added 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
//! [EventTriggers] `triggeredPageRefresh`: the ECU changed a page by itself
//! (a preset, a Grab, an ETB calibration) and says so through a live
//! channel. Without this the app kept showing - and would later write back -
//! the values from before.

use crate::{set_conn_lock_holder, AppState};
use tauri::Emitter;

/// The `(page, expression)` pairs of the loaded INI, pages 0-based.
#[tauri::command]
pub async fn get_page_refresh_triggers(
    state: tauri::State<'_, AppState>,
) -> Result<Vec<(u8, String)>, String> {
    let def = state.definition.lock().await;
    Ok(def
        .as_ref()
        .map(|d| d.page_refresh_triggers.clone())
        .unwrap_or_default())
}

/// Read `page` from the ECU into the cache and the tune, drop the tune's
/// stored values for that page (they are stale now), and refresh the views.
#[tauri::command]
pub async fn refresh_page_from_ecu(
    app: tauri::AppHandle,
    state: tauri::State<'_, AppState>,
    page: u8,
) -> Result<(), String> {
    let data = {
        set_conn_lock_holder("refresh_page_from_ecu");
        let mut conn = state.connection.lock().await;
        let r = match conn.as_mut() {
            Some(c) => c.read_page(page).map_err(|e| e.to_string()),
            None => Err("Not connected".to_string()),
        };
        drop(conn);
        set_conn_lock_holder("(none)");
        r?
    };
    let names: Vec<String> = {
        let def = state.definition.lock().await;
        def.as_ref()
            .map(|d| {
                d.constants
                    .values()
                    .filter(|c| !c.is_pc_variable && c.page == page)
                    .map(|c| c.name.clone())
                    .collect()
            })
            .unwrap_or_default()
    };
    if let Some(cache) = state.tune_cache.lock().await.as_mut() {
        cache.load_page(page, data.clone());
    }
    if let Some(tune) = state.current_tune.lock().await.as_mut() {
        tune.pages.insert(page, data);
        for n in &names {
            tune.constants.remove(n);
        }
    }
    // The project file does not hold these values yet.
    *state.tune_modified.lock().await = true;
    let _ = app.emit("tune:loaded", "ecu-page-refresh");
    Ok(())
}
