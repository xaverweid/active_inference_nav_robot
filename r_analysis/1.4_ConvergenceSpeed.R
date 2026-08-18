library(ggplot2)
library(tidyr)
library(dplyr)
library(extrafont)
library(readxl)
library(stringr)
library(purrr)
library(readr)
library(emmeans)


# --- STEP 1: INITIALIZE FONTS ---
loadfonts(device = "pdf", quiet = TRUE) 

# 2. DEFINE PATHS & THRESHOLDS 
# Using "../" steps out of DataAnalysis/ and into the root GitHub directory
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


raw_speed_data <- master_data %>%
  # 1. Clear out the unconstrained baseline as requested
  filter(Algorithm != "Unconstrained Random") %>%
  # 2. Group by environment and unique trial to isolate the end-state of each run
  group_by(Map, Duration, Algorithm, run_id) %>% 
  filter(step == max(step)) %>% 
  ungroup() %>%
  # 3. Apply your strict error thresholds to isolate True Successes exclusively
  
  filter(position_error <= POS_ERROR_THRESHOLD & rotational_error <= ROT_ERROR_THRESHOLD) %>%
  # 4. Rename the step counter to reflect total convergence cost per trial
  rename(Total_Steps = step)

# --- STEP 3.5: AGGREGATE SUMMARY STATISTICS FOR PLOTTING ---
algo_order <- c("Standard AIC", "Deep AIC", "Full-Distribution AIC", "Purely Epistemic AIC", "D-Optimality", "Constrained Random")

master_speed_data <- raw_speed_data %>%
  group_by(Map, Duration, Algorithm) %>%
  summarise(
    Mean_Steps = mean(Total_Steps, na.rm = TRUE),
    Std_Steps  = sd(Total_Steps, na.rm = TRUE),
    N_Trials   = n(),
    # Calculate Standard Error: SE = SD / sqrt(N)
    SE_Steps   = Std_Steps / sqrt(N_Trials),
    
    # Calculate 95% Confidence Interval bounds
    CI_Lower   = Mean_Steps - (1.96 * SE_Steps),
    CI_Upper   = Mean_Steps + (1.96 * SE_Steps),
    .groups    = "drop"
  ) %>%
  filter(Algorithm %in% algo_order) %>%
  mutate(
    Algorithm = factor(Algorithm, levels = algo_order),
    Map = factor(Map, levels = c("Indoor Map", "H-Map", "Expanded H-Map"))
    )


significance_labels <- master_speed_data %>%
  mutate(
    Label = case_when(
      # === H-MAP / 1s PANEL ===
      Map == "H-Map" & Duration == "1s" & Algorithm == "Purely Epistemic AIC" ~ "**", # Fastest overall
      Map == "H-Map" & Duration == "1s" & Algorithm == "Standard AIC"         ~ "**",          # Sig faster than baseline
      Map == "H-Map" & Duration == "1s" & Algorithm == "Deep AIC"             ~ "*",          # Sig faster than baseline
      
      # === INDOOR MAP / 1s PANEL ===
      Map == "Indoor Map" & Duration == "1s" & Algorithm == "Standard AIC"         ~ "***",       # Sig faster than baseline
      Map == "Indoor Map" & Duration == "1s" & Algorithm == "Deep AIC"             ~ "***",      # Sig faster than baseline
      Map == "Indoor Map" & Duration == "1s" & Algorithm == "Full-Distribution AIC" ~ "***",     # Sig faster than baseline
      Map == "Indoor Map" & Duration == "1s" & Algorithm == "Purely Epistemic AIC"  ~ "***",     # Sig faster than baseline
      
      # === 5s PANELS  ===
      # === INDOOR MAP / 5s PANEL ===
      Map == "Indoor Map" & Duration == "5s" & Algorithm == "Standard AIC"         ~ "**",       # Sig faster than baseline
      Map == "Indoor Map" & Duration == "5s" & Algorithm == "Deep AIC"             ~ "***",      # Sig faster than baseline
      Map == "Indoor Map" & Duration == "5s" & Algorithm == "Full-Distribution AIC" ~ "**",     # Sig faster than baseline
      Map == "Indoor Map" & Duration == "5s" & Algorithm == "Purely Epistemic AIC"  ~ "***",     # Sig faster than baselin
      
      # === INDOOR MAP / 1s PANEL ===
      Map == "H-Map" & Duration == "5s" & Algorithm == "Standard AIC"         ~ "***",       # Sig faster than baseline
      Map == "H-Map" & Duration == "5s" & Algorithm == "Deep AIC"             ~ "***",      # Sig faster than baseline
      Map == "H-Map" & Duration == "5s" & Algorithm == "Full-Distribution AIC" ~ "**",     # Sig faster than baseline
      Map == "H-Map" & Duration == "5s" & Algorithm == "Purely Epistemic AIC"  ~ "***",     # Sig faster than baselin
      TRUE ~ "" # Blank for non-significant differences
    ),
    # Dynamically place the label just above the positive Standard Deviation line
    Y_Pos = CI_Upper + (Mean_Steps * 0.05)
    )

# --- STEP 5: GENERATE THE VISUAL MATRIX WITH SIGNIFICANCE OVERLAYS ---
grid_speed_plot <- ggplot(master_speed_data, aes(x = Algorithm, y = Mean_Steps)) +
  
  # Base bars and error markings
  geom_col(fill = "#56B4E9", width = 0.65, color = NA) + 

  geom_errorbar(
    aes(ymin = CI_Lower, ymax = CI_Upper),
    width = 0.2,
    color = "#2C3E50", 
    linewidth = 0.5
  ) +
  
  geom_text(
    data = significance_labels,
    aes(x = Algorithm, y = Y_Pos, label = Label),
    family = "Arial",
    fontface = "bold",
    size = 3.5,
    vjust = 0,
    color = "#2C3E50"
  ) +
  
  facet_grid(Duration ~ Map, scales = "free", space = "free_x") +
  
  # Expand y-axis slightly to ensure annotations don't get clipped off the top
  scale_y_continuous(expand = expansion(mult = c(0, 0.15))) +
  
  labs(
    x = "Control Algorithm",
    y = "Mean Steps to Convergence (with 95% CI)",
    caption = "* p < .05, ** p < .01, *** p < .001 (Tukey-adjusted comparisons relative to Constrained Random baseline)"
  ) +
  
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 12),
    axis.title = element_text(face = "bold", size = 12),
    axis.text = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 45, hjust = 1), 
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 12, color = "black"),
    panel.spacing = unit(1.5, "lines"),
    
    panel.border = element_blank(),
    plot.caption = element_text(size = 9, hjust = 0.5, face = "italic", color = "#555555"),
    legend.position = "none"
  )

print(grid_speed_plot)



# --- STEP 6: SAVE AS HIGH-RES VECTOR GRAPH ---
ggsave(
  filename = "1.4_Convergence_Speed_Matrix.png", 
  plot = grid_speed_plot, 
  device = "png", 
  width = 11, 
  height = 7.5, 
  units = "in",
  dpi = 600
)

# ─────────────────────────────────────────────────────────────────────────────
# STEP 7: STATISTICAL INFERENCE (NESTED SPEED ANALYSIS VIA ANCOVA/LM)
# ─────────────────────────────────────────────────────────────────────────────
library(emmeans)

cat("\n======================================================\n")
cat("RUNNING UNIFIED LINEAR CONVERGENCE MODELS")
cat("\n======================================================\n")

# Ensure factors are appropriately designated
raw_speed_data <- raw_speed_data %>%
  mutate(
    Algorithm = factor(Algorithm, levels = algo_order),
  )

# ─────────────────────────────────────────────────────────────────────────────
# 1. INDOOR MAP MODELS & PAIRWISE COMPARISONS
# ─────────────────────────────────────────────────────────────────────────────


# --- Indoor Map: 1s Duration ---
glm_speed_indoor_1s <- lm(Total_Steps ~ Algorithm, 
                          data = subset(raw_speed_data, Map == "Indoor Map" & Duration == "1s"))

comp_indoor_1s <- emmeans(glm_speed_indoor_1s, pairwise ~ Algorithm)


# --- Indoor Map: 5s Duration ---
glm_speed_indoor_5s <- lm(Total_Steps ~ Algorithm, 
                          data = subset(raw_speed_data, Map == "Indoor Map" & Duration == "5s"))

comp_indoor_5s <- emmeans(glm_speed_indoor_5s, pairwise ~ Algorithm)


# ─────────────────────────────────────────────────────────────────────────────
# 2. H-MAP MODELS & PAIRWISE COMPARISONS
# ─────────────────────────────────────────────────────────────────────────────

# --- H-Map: 1s Duration ---
glm_speed_hmap_1s  <- lm(Total_Steps ~ Algorithm, 
                         data = subset(raw_speed_data, Map == "H-Map" & Duration == "1s"))

comp_hmap_1s  <- emmeans(glm_speed_hmap_1s, pairwise ~ Algorithm)


# --- H-Map: 5s Duration ---
glm_speed_hmap_5s  <- lm(Total_Steps ~ Algorithm, 
                         data = subset(raw_speed_data, Map == "H-Map" & Duration == "5s"))

comp_hmap_5s  <- emmeans(glm_speed_hmap_5s, pairwise ~ Algorithm)


# ─────────────────────────────────────────────────────────────────────────────
# 3. DISPLAY RESULTS 
# ─────────────────────────────────────────────────────────────────────────────

cat("\n--- Pairwise Contrasts: Indoor-Map 1s (Tukey-Adjusted p-values) ---\n")
print(comp_indoor_1s$emmeans)
print(comp_indoor_1s$contrasts)

cat("\n--- Pairwise Contrasts: Indoor-Map 5s (Tukey-Adjusted p-values) ---\n")
print(comp_indoor_5s$contrasts)
print(comp_indoor_5s$emmeans)

cat("\n--- Pairwise Contrasts: H-Map 1s (Tukey-Adjusted p-values) ---\n")
print(comp_hmap_1s$contrasts)
print(comp_hmap_1s$emmeans)

cat("\n--- Pairwise Contrasts: H-Map 5s (Tukey-Adjusted p-values) ---\n")
print(comp_hmap_5s$contrasts)
print(comp_hmap_5s$emmeans)

# --- STEP 8: FILTER TO 5s DURATION ONLY ---
master_speed_data_5s <- master_speed_data %>%
  filter(Duration == "5s")

significance_labels_5s <- significance_labels %>%
  filter(Duration == "5s")

# --- STEP 9: GENERATE THE 5s-ONLY VISUAL MATRIX WITH SIGNIFICANCE OVERLAYS ---
grid_speed_plot_5s <- ggplot(master_speed_data_5s, aes(x = Algorithm, y = Mean_Steps)) +
  
  # Base bars and error markings
  geom_col(fill = "#56B4E9", width = 0.65, color = NA) + 
  
  geom_errorbar(
    aes(ymin = CI_Lower, ymax = CI_Upper),
    width = 0.2,
    color = "#2C3E50", 
    linewidth = 0.5
  ) +
  
  geom_text(
    data = significance_labels_5s,
    aes(x = Algorithm, y = Y_Pos, label = Label),
    family = "Arial",
    fontface = "bold",
    size = 3.5,
    vjust = 0,
    color = "#2C3E50"
  ) +
  
  # Only faceting by Map now, since Duration is fixed to 5s
  facet_grid(~ Map, scales = "free", space = "free_x") +
  
  # Expand y-axis slightly to ensure annotations don't get clipped off the top
  scale_y_continuous(expand = expansion(mult = c(0, 0.15))) +
  
  labs(
    x = "Control Algorithm",
    y = "Mean Steps to Convergence (with 95% CI)",
    caption = "* p < .05, ** p < .01, *** p < .001 (Tukey-adjusted comparisons relative to Constrained Random baseline)"
  ) +
  
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 12),
    axis.title = element_text(face = "bold", size = 12),
    axis.title.y = element_text(margin = margin(r = 12)),  # <-- push y title away from axis
    axis.text = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 45, hjust = 1), 
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 12, color = "black"),
    panel.spacing = unit(1.5, "lines"),
    
    panel.border = element_blank(),
    plot.caption = element_text(size = 9, hjust = 0.5, face = "italic", color = "#555555"),
    legend.position = "none"
  )

print(grid_speed_plot_5s)

# --- STEP 10: SAVE AS HIGH-RES VECTOR GRAPH (5s ONLY) ---
ggsave(
  filename = "1.4_ConvergenceSpeed_Matrix_5s.png", 
  plot = grid_speed_plot_5s, 
  device = "png", 
  width = 11,   
  height = 5,  # shorter since we dropped the 1s row entirely
  units = "in",
  dpi = 600
)



