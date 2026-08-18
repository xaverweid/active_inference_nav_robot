# ─────────────────────────────────────────────────────────────────────────────
# EPISTEMIC PRAGMATIC TRAJECTORY ILLUSTRATION
# ─────────────────────────────────────────────────────────────────────────────
library(tidyverse)
library(extrafont)

# --- STEP 1: INITIALIZE FONTS ---
loadfonts(device = "pdf", quiet = TRUE) 

# 2. DEFINE PATHS & THRESHOLDS 
DATA_DIR          <- "../figures"  
POS_ERROR_THRESHOLD <- 0.5   
ROT_ERROR_THRESHOLD <- 0.5   

target_paths <- c(
  file.path(DATA_DIR, "my_map"),
  file.path(DATA_DIR, "h_map")
)

print("Discovering master time-series files strictly within target directories...")
file_list <- list.files(
  path = target_paths, 
  pattern = "^master_timeseries_.*\\.csv$", 
  full.names = TRUE, 
  recursive = TRUE
)
print(file_list)
if (length(file_list) == 0) stop("No matching master time-series files found in the specified map folders!")

# --- STEP 3: INGEST AND VALIDATE USING PARALLEL SUMMARY LOGS ---
master_data <- file_list %>%
  map_df(function(file_path) {
    # 1. Break down folder hierarchy to look up the data logs accurately
    # Structure: "../figures/[map_dir]/[dur_dir]/[algo_dir]/master_timeseries_..."
    path_parts <- str_split(file_path, "/")[[1]]
    
    map_dir  <- path_parts[3]  # "my_map", "h_map", or "h_map_very_large_RandomStart"
    dur_dir  <- path_parts[4]  # "1" or "5"
    algo_dir <- path_parts[5]  # e.g., "active_inf_5", "random_walk"
    
    # 2. Extract shorthand naming parts from the filename for algorithm mapping
    file_name  <- basename(file_path)
    clean_name <- str_remove(file_name, "master_timeseries_") %>% str_remove("\\.csv")
    parts      <- str_split(clean_name, "_")[[1]]
    raw_algo   <- parts[length(parts)]    
    
    # 3. Read the raw time-series tracking data (Only once!)
    df_ts <- read_csv(file_path, show_col_types = FALSE)
    df_ts <- df_ts %>% 
      mutate(run_id = as.numeric(run_id) + 1)
    
    # 4. Reconstruct parallel path to look for the summary validation logs
    parallel_data_path <- file.path("../data", map_dir, dur_dir, algo_dir)
    summary_file <- list.files(
      path       = parallel_data_path, 
      pattern    = "^summary_.*\\.(xlsx|csv)$", 
      full.names = TRUE
    )
    
    if (length(summary_file) == 0) {
      warning(paste("Missing summary verification file in:", parallel_data_path, "- Skipping folder."))
      return(NULL)
    }
    
    # 5. Handle both CSV and Excel summary log types safely
    if (str_detect(summary_file[1], "\\.xlsx$")) {
      df_summary <- read_excel(summary_file[1])
    } else {
      df_summary <- read_csv(summary_file[1], show_col_types = FALSE)
    }
    
    # 6. Isolate exact pose_index numbers that successfully converged
    successful_trials <- df_summary %>%
      filter(status == "SUCCESS: Convergence reached") %>%
      pull(pose_index)
    
    # 7. Keep ONLY rows belonging to ground-truth validated successes
    df_ts_clean <- df_ts %>% 
      filter(run_id %in% successful_trials)
    
    if (nrow(df_ts_clean) == 0) return(NULL)
    
    # 8. Assign clean factors using the correct directory/string variables
    df_ts_clean$Map <- case_match(map_dir,
                                  "my_map"                       ~ "Indoor Map", 
                                  "h_map"                        ~ "H-Map", 
                                  "h_map_very_large_RandomStart" ~ "H-Map Very Large",
                                  .default                       = map_dir
    )
    
    df_ts_clean$Duration <- case_match(dur_dir,
                                       "1" ~ "1s",
                                       "5" ~ "5s",
                                       .default = dur_dir
    )
    
    df_ts_clean$Algorithm <- case_match(raw_algo,
                                        "5"         ~ "Standard AIC", 
                                        "min"       ~ "Purely Epistemic AIC", 
                                        "h3"        ~ "Deep AIC",
                                        "500"       ~ "Full-Distribution AIC", 
                                        "particle"  ~ "D-Optimality",
                                        "walk"      ~ "Constrained Random", 
                                        "avoidance" ~ "Unconstrained Random", 
                                        .default    = raw_algo
    )
    
    # Return the clean, filtered data frame
    return(df_ts_clean)
  })



# Define the 5 core algorithms for analysis
algo_order <- c("Standard AIC", "Deep AIC", "Full-Distribution AIC", "Purely Epistemic AIC", "D-Optimality", "Constrained Random")


# Filter strictly for the 3 Active Inference Variants
aic_variants <- c("Standard AIC", "Deep AIC", "Full-Distribution AIC")

master_data <- master_data %>%
  filter(Algorithm %in% aic_variants) %>%
  mutate(
    Algorithm = factor(Algorithm, levels = aic_variants),
    Map       = factor(Map, levels = c("Indoor Map", "H-Map"))
  )

# 4. FILTER FOR TRUE SUCCESSES ONLY
print("Filtering for successful localization trials...")
successful_runs <- master_data %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  filter(step == max(step)) %>% 
  filter(position_error <= POS_ERROR_THRESHOLD & rotational_error <= ROT_ERROR_THRESHOLD) %>%
  select(Map, Duration, Algorithm, run_id) %>%
  ungroup()

plot_ready_df <- master_data %>%
  semi_join(successful_runs, by = c("Map", "Duration", "Algorithm", "run_id"))

# 5. FIX UNEQUAL RUN LENGTHS (LOCF PADDING WITH RAW VARIABLES)
print("Padding shorter runs (LOCF)...")
max_1s <- max(plot_ready_df$step[plot_ready_df$Duration == "1s"], na.rm = TRUE)
max_5s <- max(plot_ready_df$step[plot_ready_df$Duration == "5s"], na.rm = TRUE)

plot_padded_df <- plot_ready_df %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  group_modify(~ {
    target_max <- if (.y$Duration == "1s") max_1s else max_5s
    .x %>% complete(step = 0:target_max) %>% fill(epistemic, pragmatic, .direction = "down")
  }) %>%
  ungroup()

# 6. CALCULATE CLEAN VALUE TREND LINES
summary_trajectory <- plot_padded_df %>%
  group_by(Map, Duration, Algorithm, step) %>%
  summarize(
    mean_epistemic = mean(epistemic, na.rm = TRUE),
    mean_pragmatic = mean(pragmatic, na.rm = TRUE),
    .groups = "drop"
  )

# 7. PIVOT TO LONG FORMAT
print("Reshaping data for overlay matrix layout...")
summary_long <- summary_trajectory %>%
  pivot_longer(
    cols = c(mean_epistemic, mean_pragmatic),
    names_to = "Metric",
    values_to = "Value"
  ) %>%
  mutate(
    Metric = case_match(Metric,
                        "mean_epistemic" ~ "Epistemic Value",
                        "mean_pragmatic" ~ "Pragmatic Value"
    ),
    Metric = factor(Metric, levels = c(
      "Epistemic Value", 
      "Pragmatic Value"
    ))
  )

# ─────────────────────────────────────────────────────────────────────────────
# 8. GENERATE MASTER PLOT (APA Solid Color Overlay)
# ─────────────────────────────────────────────────────────────────────────────
print("Rendering master illustration...")

overlay_dynamics_plot <- ggplot(summary_long, aes(x = step, y = Value, color = Metric)) +
  geom_line(linewidth = 0.8, alpha = 0.9, linetype = "solid") + 
  
  # Rows = Algorithm, Columns = Duration + Map Combo (Free scales for independent X/Y ranges)
  facet_grid(Algorithm ~ Duration + Map, scales = "free", switch = "y") +
  
  labs(x = "Simulation Step", y = NULL, color = "Value Component") +
  
  # ─── APA COLOR SELECTION ───
  scale_color_manual(values = c(
    "Epistemic Value" = "#0072B2",  # Okabe-Ito Secure Blue (Information)
    "Pragmatic Value" = "#D55E00"   # Okabe-Ito Vermilion (Safety/Action)
  )) +
  
  theme_classic(base_size = 11, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 11),
    axis.title.x = element_text(face = "bold", size = 12, margin = margin(t = 12)),
    axis.text = element_text(color = "black", size = 9),
    axis.text.x = element_text(angle = 0, hjust = 0.5), 
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 11),
    strip.text.y.left = element_text(angle = 90, margin = margin(r = 10)), 
    strip.placement = "outside", 
    
    panel.spacing.x = unit(1.5, "lines"), 
    panel.spacing.y = unit(1.5, "lines"), 
    
    # ─── MATCHED ACADEMIC LEGEND LAYOUT (Right-Side Boxed) ───
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 11, margin = margin(b = 8)),
    legend.text = element_text(size = 10),
    
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )

# View layout in RStudio
print(overlay_dynamics_plot)


# Save high-res asset to disk
ggsave(
  filename = "2.2_EpistemicPragmaticDynamics.png", 
  plot = overlay_dynamics_plot, 
  device = "png", 
  width = 13, 
  height = 7.0, 
  units = "in",
  dpi = 600, 
  type = "cairo"
)

# --- STEP 9: FILTER TO 5s DURATION ONLY ---
summary_long_5s <- summary_long %>%
  filter(Duration == "5s")

# --- STEP 10: GENERATE THE 5s-ONLY OVERLAY PLOT ---
overlay_dynamics_plot_5s <- ggplot(summary_long_5s, aes(x = step, y = Value, color = Metric)) +
  geom_line(linewidth = 0.8, alpha = 0.9, linetype = "solid") + 
  
  # Rows = Algorithm, Columns = Map only (Duration is now fixed to 5s)
  facet_grid(Algorithm ~ Map, scales = "free", switch = "y") +
  
  labs(x = "Simulation Step", y = NULL, color = "Value Component") +
  
  scale_color_manual(values = c(
    "Epistemic Value" = "#0072B2",  # Okabe-Ito Secure Blue (Information)
    "Pragmatic Value" = "#D55E00"   # Okabe-Ito Vermilion (Safety/Action)
  )) +
  
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 12),
    axis.title.x = element_text(face = "bold", size = 12, margin = margin(t = 12)),
    axis.text = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 0, hjust = 0.5), 
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 12),
    strip.text.y.left = element_text(angle = 90, margin = margin(r = 10)), 
    strip.placement = "outside", 
    
    panel.spacing.x = unit(1.5, "lines"), 
    panel.spacing.y = unit(1.5, "lines"), 
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 12, margin = margin(b = 8)),
    legend.text = element_text(size = 11),
    
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10),
    
    plot.margin = margin(t = 5.5, r = 5.5, b = 5.5, l = 12)
  )
print(overlay_dynamics_plot_5s)

# --- STEP 11: SAVE AS HIGH-RES VECTOR GRAPH (5s ONLY) ---
ggsave(
  filename = "2.2_EpistemicPragmaticDynamics_5s.png", 
  plot = overlay_dynamics_plot_5s, 
  device = "png", 
  width = 13, 
  height = 7.0, 
  units = "in",
  dpi = 600, 
  type = "cairo"
)

