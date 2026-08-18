# ─────────────────────────────────────────────────────────────────────────────
# THESIS ANALYSIS: SYMMETRY RESOLUTION & BIMODALITY LIFESPAN
# ─────────────────────────────────────────────────────────────────────────────
library(tidyverse)
library(extrafont)

# --- STEP 1: INITIALIZE FONTS ---
loadfonts(device = "pdf", quiet = TRUE) 

# 2. DEFINE PATHS & THRESHOLDS 
OUTPUT_DIR          <- "/Users/Xaver/Desktop/MasterThesis/active_inference_nav_robot/figures"  
POS_ERROR_THRESHOLD <- 0.5   
ROT_ERROR_THRESHOLD <- 0.5   

# Lock down target directories to isolate data and prevent contamination
target_paths <- c(
  file.path(OUTPUT_DIR, "my_map"),
  file.path(OUTPUT_DIR, "h_map")
)

print("Discovering master time-series files strictly within target directories...")
file_list <- list.files(
  path = target_paths, 
  pattern = "^master_timeseries_.*\\.csv$", 
  full.names = TRUE, 
  recursive = TRUE
)

if (length(file_list) == 0) stop("No matching master time-series files found!")

# 3. LOAD, MERGE, AND CLEAN CSV FILES
master_data <- file_list %>%
  map_df(function(file_path) {
    df <- read_csv(file_path, show_col_types = FALSE)
    file_name <- basename(file_path)
    clean_name <- str_remove(file_name, "master_timeseries_") %>% str_remove("\\.csv")
    parts <- str_split(clean_name, "_")[[1]]
    
    raw_map  <- paste(parts[1:2], collapse = "_") 
    df$Duration  <- grep("s$", parts, value = TRUE)   
    raw_algo <- parts[length(parts)]              
    
    df$Map <- case_match(raw_map,
                         "my_map" ~ "Indoor Map", "indoor_map" ~ "Indoor Map", "h_map" ~ "H-Map", .default = raw_map
    )
    df$Algorithm <- case_match(raw_algo,
                               "5" ~ "Standard AIC", "min" ~ "Purely Epistemic AIC", "h3" ~ "Deep AIC",
                               "500" ~ "Full-Distribution AIC", "particle" ~ "D-Optimality",
                               "walk" ~ "Constrained Random", "avoidance" ~ "Unconstrained Random", .default = raw_algo
    )
    return(df)
  })

# Define the 5 target algorithms for the symmetry investigation
target_algos <- c("Standard AIC", "Deep AIC", "Full-Distribution AIC", "D-Optimality", "Constrained Random")

master_filtered <- master_data %>%
  filter(Algorithm %in% target_algos) %>%
  mutate(
    Algorithm = factor(Algorithm, levels = target_algos),
    Map       = factor(Map, levels = c("Indoor Map", "H-Map"))
  )

# 4. FILTER STRICTLY FOR TRUE SUCCESSES ONLY
print("Filtering for successful localization trials...")
successful_runs <- master_filtered %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  filter(step == max(step)) %>% 
  filter(position_error <= POS_ERROR_THRESHOLD & rotational_error <= ROT_ERROR_THRESHOLD) %>%
  select(Map, Duration, Algorithm, run_id) %>%
  ungroup()

# Retain active, unpadded trajectories for successful runs
active_runs_df <- master_filtered %>%
  semi_join(successful_runs, by = c("Map", "Duration", "Algorithm", "run_id"))

# 5. CALCULATE BIMODAL PERCENTAGE PER INDIVIDUAL RUN
print("Calculating run-level ambiguity profiles...")
run_summary <- active_runs_df %>%
  group_by(Map, Duration, Algorithm, run_id) %>%
  summarize(
    total_active_steps = n(),
    steps_in_bimodal   = sum(is_bimodal == 1, na.rm = TRUE),
    pct_time_bimodal   = (steps_in_bimodal / total_active_steps) * 100,
    .groups = "drop"
  )

# 6. AGGREGATE COMPREHENSIVE STATISTICS PER CONDITION
print("Generating descriptive summary matrices...")
condition_stats <- run_summary %>%
  group_by(Map, Duration, Algorithm) %>%
  summarize(
    mean_pct_bimodal = mean(pct_time_bimodal, na.rm = TRUE),
    sd_pct_bimodal   = sd(pct_time_bimodal, na.rm = TRUE),
    sample_size_n    = n(),
    se_pct_bimodal   = sd_pct_bimodal / sqrt(sample_size_n),
    .groups = "drop"
  )

# Output summary metrics directly to console for quick review
print("======= SUMMARY STATISTICS =======")
print(condition_stats %>% filter(Map == "H-Map")) # Prioritizing your H-Map focus

# ─────────────────────────────────────────────────────────────────────────────
# 7. GENERATE EXPLORATORY BOXPLOT
# ─────────────────────────────────────────────────────────────────────────────
print("Rendering exploratory distribution profiles...")

bimodal_boxplot <- ggplot(run_summary, aes(x = Algorithm, y = pct_time_bimodal, fill = Algorithm)) +
  geom_boxplot(outlier.shape = 16, outlier.size = 1.5, alpha = 0.8, lwd = 0.5) +
  facet_grid(Duration ~ Map) +
  labs(
    x = NULL,
    y = "Percentage of Active Run Spent in Bimodal State (%)",
    title = "Ambiguity Lifespan Distribution Across Environmental Layouts"
  ) +
  scale_fill_brewer(palette = "Set2") +
  theme_classic(base_size = 11, base_family = "Times New Roman") +
  theme(
    text = element_text(family = "Times New Roman"),
    axis.text.x = element_text(angle = 45, hjust = 1),
    strip.background = element_blank(),
    strip.text = element_text(face = "bold", size = 11),
    legend.position = "none",
    panel.spacing = unit(1.2, "lines")
  )

print(bimodal_boxplot)

# Save the diagnostic plot
ggsave(
  filename = "4_BimodalLifespanDistribution.png",
  plot = bimodal_boxplot,
  path = OUTPUT_DIR,
  device = "png",
  width = 11,
  height = 6.5,
  dpi = 300,
  type = "cairo"
)