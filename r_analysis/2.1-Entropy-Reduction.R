# ─────────────────────────────────────────────────────────────────────────────
# Entropy Reduction Analysis
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

algo_palette <- c(
  "Standard AIC"          = "#4A6FA5",
  "Deep AIC"              = "#D4A373",
  "Full-Distribution AIC" = "#B33939",
  "Purely Epistemic AIC"  = "#2C3E50",
  "D-Optimality"          = "#9B86A6",
  "Constrained Random"    = "#1ABC9C"
)

# Filter out the excluded models and enforce structural ordering
master_data <- master_data %>%
  filter(Algorithm %in% algo_order) %>%
  mutate(
    Algorithm = factor(Algorithm, levels = algo_order),
    Map       = factor(Map, levels = c("Indoor Map", "H-Map"))
  )

# 4. FILTER FOR TRUE SUCCESSES ONLY
successful_runs <- master_data %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  filter(step == max(step)) %>% 
  filter(position_error <= POS_ERROR_THRESHOLD & rotational_error <= ROT_ERROR_THRESHOLD) %>%
  select(Map, Duration, Algorithm, run_id) %>%
  ungroup()

plot_ready_df <- master_data %>%
  semi_join(successful_runs, by = c("Map", "Duration", "Algorithm", "run_id"))

# ═════════════════════════════════════════════════════════════════════════════
# PART A: MATHEMATICAL INFERENCE (STATISTICAL ANALYSIS MODELLING)
# ═════════════════════════════════════════════════════════════════════════════

print("Configuring LMM structural data matrices...")
lmm_data <- plot_ready_df %>%
  mutate(
    Algorithm = relevel(factor(Algorithm, levels = algo_order), ref = "Standard AIC"),
    Duration  = factor(Duration)
  )

lmm_indoor <- lmm_data %>% filter(Map == "Indoor Map")
lmm_hmap   <- lmm_data %>% filter(Map == "H-Map")

print("Executing Linear Mixed-Effects Models (Satterthwaite approximation)...")
library(lme4)
library(lmerTest)


# Shannon Entropy Decay Rate Models
# ── INDOOR MAP ──────────────────────────────────────────────────────────────
lmer_shannon_indoor_1s <- lmer(shannon_entropy ~ log(step + 1) * Algorithm + (1 | run_id), 
                               data = subset(lmm_indoor, Duration == "1s"), REML = TRUE)

lmer_shannon_indoor_5s <- lmer(shannon_entropy ~ log(step + 1) * Algorithm + (1 | run_id), 
                               data = subset(lmm_indoor, Duration == "5s"), REML = TRUE)

# ── H-MAP ───────────────────────────────────────────────────────────────────
lmer_shannon_hmap_1s   <- lmer(shannon_entropy ~ log(step + 1) * Algorithm + (1 | run_id), 
                               data = subset(lmm_hmap, Duration == "1s"), REML = TRUE)

lmer_shannon_hmap_5s   <- lmer(shannon_entropy ~ log(step + 1) * Algorithm + (1 | run_id), 
                               data = subset(lmm_hmap, Duration == "5s"), REML = TRUE)


# --- 2. GLM/LM: TERMINAL CEILING ASYMPTOTE (Floor Entropy States) ---
print("Isolating final step coordinates for floor-asymptote evaluation...")
terminal_df <- plot_ready_df %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  filter(step == max(step)) %>%
  ungroup() %>%
  mutate(
    Algorithm = relevel(factor(Algorithm, levels = algo_order), ref = "Standard AIC"),
    Duration  = factor(Duration)
  )

terminal_indoor <- terminal_df %>% filter(Map == "Indoor Map")
terminal_hmap   <- terminal_df %>% filter(Map == "H-Map")

print("Fitting asymptotic linear response models...")

# 1. Terminal Shannon Asymptote Floor Models
# ── INDOOR MAP ──────────────────────────────────────────────────────────────
glm_shannon_indoor_1s <- lm(shannon_entropy ~ Algorithm, 
                            data = subset(terminal_indoor, Duration == "1s"))

glm_shannon_indoor_5s <- lm(shannon_entropy ~ Algorithm, 
                            data = subset(terminal_indoor, Duration == "5s"))

# ── H-MAP ───────────────────────────────────────────────────────────────────
glm_shannon_hmap_1s   <- lm(shannon_entropy ~ Algorithm, 
                            data = subset(terminal_hmap, Duration == "1s"))

glm_shannon_hmap_5s   <- lm(shannon_entropy ~ Algorithm, 
                            data = subset(terminal_hmap, Duration == "5s"))

# ═════════════════════════════════════════════════════════════════════════════
# PART B: VISUAL COMPOSITION GENERATION (PLOT PIPELINE)
# ═════════════════════════════════════════════════════════════════════════════


print("Executing Last Observation Carried Forward (LOCF) array padding...")
max_1s <- max(plot_ready_df$step[plot_ready_df$Duration == "1s"], na.rm = TRUE)
max_5s <- max(plot_ready_df$step[plot_ready_df$Duration == "5s"], na.rm = TRUE)

plot_padded_df <- plot_ready_df %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  group_modify(~ {
    target_max <- if (.y$Duration == "1s") max_1s else max_5s
    .x %>% complete(step = 0:target_max) %>% fill(shannon_entropy, .direction = "down")
  }) %>%
  ungroup()

print("Compiling macro trend averages...")
summary_long <- plot_padded_df %>%
  group_by(Map, Duration, Algorithm, step) %>%
  summarize(
    mean_shannon  = mean(shannon_entropy, na.rm = TRUE),
    .groups = "drop"
  ) %>%
  pivot_longer(
    cols = c(mean_shannon),
    names_to = "Metric",
    values_to = "Value"
  ) %>%
  mutate(
    Metric = case_match(Metric, "mean_shannon" ~ "Shannon Entropy"),
    Metric = factor(Metric, levels = c("Shannon Entropy"))
  )

print("Rendering unified vector trajectory illustration...")
traject_eff_plot <- ggplot(summary_long, aes(x = step, y = Value, color = Algorithm)) +
  geom_line(linewidth = 0.8) + 
  facet_grid(Metric ~ Duration + Map, scales = "free", switch = "y") +
  labs(x = "Simulation Step", y = NULL, color = "Control Logic") +
  scale_color_manual(values = algo_palette) + 
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
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 11, margin = margin(b = 8)),
    legend.text = element_text(size = 10),
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )

print(traject_eff_plot)

ggsave(
  filename = file.path("2.1_EntropyReduction.png"), 
  plot = traject_eff_plot, 
  device = "png", 
  width = 13, 
  height = 5, 
  units = "in",
  dpi = 600, 
  type = "cairo"
)

# =============================================================================
# PART C: CONSOLE DIAGNOSTIC REPORT PRINTINGS (SPLIT BY SENSING DURATION)
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1: LINEAR MIXED MODELS (TRAJECTORY DECAY RATES)
# ─────────────────────────────────────────────────────────────────────────────

cat("--- 1. LMM: SHANNON ENTROPY DECAY RATE (INDOOR MAP - 1s) ---\n")
print(summary(lmer_shannon_indoor_1s))

cat("\n--- 2. LMM: SHANNON ENTROPY DECAY RATE (INDOOR MAP - 5s) ---\n")
print(summary(lmer_shannon_indoor_5s))


cat("\n--- 5. LMM: SHANNON ENTROPY DECAY RATE (H-MAP - 1s) ---\n")
print(summary(lmer_shannon_hmap_1s))

cat("\n--- 6. LMM: SHANNON ENTROPY DECAY RATE (H-MAP - 5s) ---\n")
print(summary(lmer_shannon_hmap_5s))



# ─────────────────────────────────────────────────────────────────────────────
# SECTION 2: GENERAL LINEAR MODELS (TERMINAL ASYMPTOTE FLOORS)
# ─────────────────────────────────────────────────────────────────────────────

cat("\n--- 9. GLM: TERMINAL SHANNON ASYMPTOTE FLOOR (INDOOR MAP - 1s) ---\n")
print(summary(glm_shannon_indoor_1s))

cat("\n--- 10. GLM: TERMINAL SHANNON ASYMPTOTE FLOOR (INDOOR MAP - 5s) ---\n")
print(summary(glm_shannon_indoor_5s))

cat("\n--- 13. GLM: TERMINAL SHANNON ASYMPTOTE FLOOR (H-MAP - 1s) ---\n")
print(summary(glm_shannon_hmap_1s))

cat("\n--- 14. GLM: TERMINAL SHANNON ASYMPTOTE FLOOR (H-MAP - 5s) ---\n")
print(summary(glm_shannon_hmap_5s))


# --- STEP D: FILTER TO 5s DURATION ONLY ---
summary_long_5s <- summary_long %>%
  filter(Duration == "5s")

# --- STEP E: RENDER THE 5s-ONLY TRAJECTORY PLOT ---
traject_eff_plot_5s <- ggplot(summary_long_5s, aes(x = step, y = Value, color = Algorithm)) +
  geom_line(linewidth = 0.8) + 
  facet_grid(Metric ~ Map, scales = "free", switch = "y") +
  labs(x = "Simulation Step", y = NULL, color = "Control Logic") +
  scale_color_manual(values = algo_palette) + 
  theme_classic(base_size = 14, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 14),
    axis.title.x = element_text(face = "bold", size = 14, margin = margin(t = 12)),
    axis.text = element_text(color = "black", size = 13),
    axis.text.x = element_text(angle = 0, hjust = 0.5), 
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 14),
    strip.text.y.left = element_text(angle = 90, margin = margin(r = 10)), 
    strip.placement = "outside", 
    
    panel.spacing.x = unit(1.5, "lines"), 
    panel.spacing.y = unit(1.5, "lines"), 
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 14, margin = margin(b = 8)),
    legend.text = element_text(size = 13),
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )
print(traject_eff_plot_5s)
ggsave(
  filename = file.path("2.1_EntropyReduction_5s.png"), 
  plot = traject_eff_plot_5s, 
  device = "png", 
  width = 10.5,  
  height = 5, 
  units = "in",
  dpi = 600, 
  type = "cairo"
)
