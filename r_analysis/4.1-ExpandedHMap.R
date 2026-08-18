library(ggplot2)
library(tidyr)
library(dplyr)
library(extrafont)
library(readxl) # Added for direct Excel ingestion

# --- STEP 1: INITIALIZE FONTS (For Arial 12pt) ---
loadfonts(device = "pdf", quiet = TRUE)

# --- STEP 2: DEFINE PATHS & IMPORT EXCEL FILES ---
DATA_DIR <- "../data" 

path_standard   <- file.path(DATA_DIR, "h_map/HMap_5s.xlsx")
path_very_large <- file.path(DATA_DIR, "h_map_very_large_RandomStart/HMapExpanded_5s.xlsx")

# Ingest raw sheets directly
df_standard_raw   <- read_xlsx(path_standard)
df_very_large_raw <- read_xlsx(path_very_large)

# Dynamically force the first column name to "Metric" to maintain pipeline consistency
colnames(df_standard_raw)[1]   <- "Metric"
colnames(df_very_large_raw)[1] <- "Metric"

# --- STEP 3: TRANSFORM WIDE TO LONG FORMAT ---
HMap_process_dataset <- function(df, map_scale_label) {
  df %>%
    # Clean string spacing/capitalization discrepancies if they exist
    mutate(Metric = trimws(tolower(Metric))) %>%
    
    # Filter out only the 5 outcome categories needed for the stacked bars
    filter(Metric %in% c("true success", "false success", "collision", "timeout", "other failure")) %>%
    
    # Pivot the algorithm columns into a single long column
    pivot_longer(cols = -Metric, names_to = "Algorithm", values_to = "Count") %>%
    
    # Label the map scale group
    mutate(Map_Scale = map_scale_label)
}

# Run the transformation pipeline
cleaned_standard   <- HMap_process_dataset(df_standard_raw, "Standard H-Map")
cleaned_very_large <- HMap_process_dataset(df_very_large_raw, "Expanded H-Map") %>%
  mutate(Count = Count / 2)

# ==============================================================================
# Standard vs Expanded H-Map plot
# ==============================================================================
plot_data_Expanded_HMap <- bind_rows(cleaned_standard, cleaned_very_large)

# --- STEP 4: STANDARDIZE LABELS, FILTER ALGORITHMS, & ORDER LAYERS ---
plot_data_Expanded_HMap <- plot_data_Expanded_HMap %>%
  # Clean up whitespace variances in algorithm text string naming from spreadsheet sheets
  mutate(Algorithm = trimws(Algorithm)) %>%
  
  # ISOLATE TARGET CONFIGURATIONS ONLY: Standard AIC, Deep AIC, Constrained Random
  # (Adjust strings here if your Excel headers spell them slightly differently)
  filter(Algorithm %in% c("Standard AIC", "Deep AIC", "Constrained Random")) %>%
  
  mutate(
    Metric = case_when(
      Metric == "true success"  ~ "True Success",
      Metric == "false success" ~ "False Success",
      Metric == "collision"     ~ "Collision",
      Metric == "timeout"       ~ "Timeout",
      Metric == "other failure" ~ "Other Failure",
      TRUE                      ~ Metric
    ),
    
    # Enforce strict left-to-right plotting sequence on the X-axis
    Algorithm = factor(Algorithm, levels = c("Standard AIC", "Deep AIC","Constrained Random")),
    
    # Sets the visual stack ordering from bottom up
    Metric = factor(Metric, levels = c("Other Failure", "Timeout", "Collision", "False Success", "True Success")),
    
    # Enforce left-to-right panel order for the map facets
    Map_Scale = factor(Map_Scale, levels = c("Standard H-Map", "Expanded H-Map")
    )
  )
# --- STEP 5: DEFINE COGNITIVE SCIENCE PALETTE ---
academic_colors <- c(
  "True Success"  = "#4A6FA5",  # Muted Slate Blue
  "False Success" = "#D4A373",  # Muted Ochre / Sand
  "Collision"     = "#B33939",  # Terracotta / Deep Red
  "Timeout"       = "#2C3E50",  # Dark Charcoal
  "Other Failure" = "#9B86A6"   # Muted Dusty Violet
)

# --- STEP 6: RENDER THE GGPLOT ---
stacked_plot_Expanded_HMap <- ggplot(plot_data_Expanded_HMap, aes(x = Algorithm, y = Count, fill = Metric)) +
  geom_bar(stat = "identity", position = "stack", width = 0.6, color = NA) +
  
  # Generates side-by-side panels comparing the two physical map scales
  facet_wrap(~Map_Scale, scales = "free_x") +
  scale_fill_manual(values = academic_colors) +
  
  # Assumes 100 trials total per run, setting raw count directly to 100% height
  scale_y_continuous(limits = c(0, 100), breaks = seq(0, 100, 20), expand = c(0, 0)) +
  
  labs(
    x = "Control Algorithm",
    y = "Percentage of Experimental Trials (%)",
    fill = "Trial Outcome"
  ) +
  
  # Strict Academic Theme Customization (Arial, 12pt)
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 12),
    axis.title = element_text(face = "bold", size = 12),
    axis.text = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 45, hjust = 1), # Cleanly angling algorithm titles
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 12, margin = margin(b = 10)),
    panel.spacing = unit(3, "lines"), 
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 12, margin = margin(b = 8)),
    legend.text = element_text(size = 11),
    
    # Draws the 0.5pt grey bounding box frame with an inner margin cushion
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )

# Output image to plot pane
print(stacked_plot_Expanded_HMap)

# --- STEP 7: EXPORT HIGH-RESOLUTION VECTOR GRAPH ---
ggsave(
  filename = "4.5.1_ExpandedH-Map_Comparison_Profiles.png", 
  plot = stacked_plot_Expanded_HMap, 
  device = "png", 
  width = 7.5, 
  height = 5.5, 
  units = "in",
  dpi = 600
)


# ==============================================================================
# STATISTICAL ANALYSIS: EXPANDED H-MAP (5s DURATION) ONLY
# ==============================================================================
library(emmeans)
library(brglm2) 

# Filter the data to isolate only the Expanded Map and the 3 target tracking configurations
df_expanded_base <- plot_data_Expanded_HMap %>%
  filter(Map_Scale == "Expanded H-Map" & 
           Algorithm %in% c("Standard AIC", "Deep AIC", "Constrained Random")) %>%
  # RESTORE TRUE SAMPLE SIZE (N=200): Reverses the /2 plotting normalization 
  # to give the GLM the required whole-number integers and eliminate warnings.
  mutate(Count = Count * 2)
# ==========================================
# 1. TIMEOUT ANALYSIS (Efficiency)
# ==========================================
# Objective: Test if algorithms differ in their likelihood to time out 
# (Excludes physical crash dropouts to isolate pure navigation efficiency)

df_eff_expanded <- df_expanded_base %>%
  filter(Metric %in% c("True Success", "False Success", "Timeout")) %>%
  mutate(Is_Timeout = ifelse(Metric == "Timeout", 1, 0))

# Run the Weighted Binomial GLM with Bias-Reduction (Firth's method via brglmFit)
model_timeout_expanded <- glm(
  Is_Timeout ~ Algorithm, 
  data = df_eff_expanded, 
  weights = Count, 
  family = binomial,
  method = "brglmFit"
)

cat("\n========================================================\n")
cat("TIMEOUT ANALYSIS: H-MAP VERY LARGE (5s)\n")
cat("========================================================\n")
pairs_timeout_expanded <- emmeans(model_timeout_expanded, pairwise ~ Algorithm, type = "response")
print(pairs_timeout_expanded$emmeans)   # Predicted probabilities of timing out
print(pairs_timeout_expanded$contrasts) # Pairwise comparison Odds Ratios & p-values


# ==========================================
# 2. PRECISION ANALYSIS (True vs. False Success)
# ==========================================
# Objective: Within runs that successfully reached convergence, does an 
# algorithm significantly improve the odds of choosing the correct topological orientation?

df_prec_expanded <- df_expanded_base %>%
  filter(Metric %in% c("True Success", "False Success")) %>%
  mutate(Is_True_Success = ifelse(Metric == "True Success", 1, 0))

# Run the Weighted Binomial GLM
model_precision_expanded <- glm(
  Is_True_Success ~ Algorithm, 
  data = df_prec_expanded, 
  weights = Count, 
  family = binomial,
  method = "brglmFit"
)

cat("\n========================================================\n")
cat("PRECISION ANALYSIS: H-MAP VERY LARGE (5s)\n")
cat("========================================================\n")
pairs_precision_expanded <- emmeans(model_precision_expanded, pairwise ~ Algorithm, type = "response")
print(pairs_precision_expanded$emmeans)   # Predicted probabilities of True Success
print(pairs_precision_expanded$contrasts) # Pairwise comparison Odds Ratios & p-values


# ==========================================
# 3. COLLISION ANALYSIS (Active Inference Only)
# ==========================================
# Objective: Compare the emergent safety profiles of the two AIC variants.
# Does a deeper temporal horizon (h=3) significantly suppress physical collisions 
# compared to standard AIC (h=1) under intrinsic pragmatic costs?

df_coll_aic <- df_expanded_base %>%
  # Isolate only the active inference architectures
  filter(Algorithm %in% c("Standard AIC", "Deep AIC")) %>%
  # Group by algorithm to evaluate collisions against ALL possible outcomes
  group_by(Algorithm, Metric) %>%
  summarise(Count = sum(Count), .groups = "drop") %>%
  # Code the binary dependent variable (Collision vs. Any Non-Collision)
  mutate(Is_Collision = ifelse(Metric == "Collision", 1, 0))

# Run the Weighted Binomial GLM with Bias-Reduction
model_collision_aic <- glm(
  Is_Collision ~ Algorithm, 
  data = df_coll_aic, 
  weights = Count, 
  family = binomial,
  method = "brglmFit"
)

cat("\n========================================================\n")
cat("COLLISION ANALYSIS: STANDARD AIC VS. DEEP AIC\n")
cat("========================================================\n")
pairs_collision_aic <- emmeans(model_collision_aic, pairwise ~ Algorithm, type = "response")
print(pairs_collision_aic$emmeans)   # Emergent crash probabilities for each variant
print(pairs_collision_aic$contrasts) # Odds ratio and p-value of the safety difference

# ==============================================================================
# CONVERGENCE SPEED ANALYSIS: EXPANDED H-MAP (5s DURATION) ONLY
# ==============================================================================
library(readr)
library(stringr)
library(purrr)
library(dplyr)
library(ggplot2)
library(emmeans)
library(extrafont)

# --- STEP 1: INITIALIZE DIRECTORIES & PATHS ---
Timeseries_DIR       <- "../figures/h_map_very_large_RandomStart/5"  
POS_ERROR_THRESHOLD  <- 0.5   
ROT_ERROR_THRESHOLD  <- 0.5   

print("Discovering master time-series files strictly within target directories...")
file_list <- list.files(
  path        = Timeseries_DIR, 
  pattern     = "^master_timeseries_.*\\.csv$", 
  full.names  = TRUE, 
  recursive   = TRUE
)

print(file_list)
if (length(file_list) == 0) {
  stop("No matching master time-series files found in the specified map folders!")
}

# --- STEP 2: INGEST AND CLEAN MASTER TIME-SERIES DATA ---
master_speed_raw <- file_list %>%
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

# --- STEP 3: FILTER FOR TRUE SUCCESSES ONLY & EXTRACT CONVERGENCE STEPS ---
raw_speed_data_filtered <- master_speed_raw %>%
  # Isolate only our 3 target algorithms
  filter(Algorithm %in% c("Standard AIC", "Deep AIC", "Constrained Random")) %>%
  
  # Group by trial run to grab the terminal state
  group_by(Map, Duration, Algorithm, run_id) %>% 
  filter(step == max(step)) %>% 
  ungroup() %>%
  
  # Strict localization thresholds: Exclude False Successes, Timeouts, and Collisions
  filter(position_error <= POS_ERROR_THRESHOLD & rotational_error <= ROT_ERROR_THRESHOLD) %>%
  rename(Total_Steps = step)

# Ensure sorting factors match your visual layout conventions
algo_order_speed <- c("Standard AIC", "Deep AIC", "Constrained Random")
raw_speed_data_filtered <- raw_speed_data_filtered %>%
  mutate(Algorithm = factor(Algorithm, levels = algo_order_speed))

# --- STEP 4: AGGREGATE SUMMARY STATISTICS FOR VISUALIZATION ---
summary_speed_data <- raw_speed_data_filtered %>%
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
  )


# --- STEP 5: GENERATE PLOT (Arial, Academic Grade) ---

speed_plot <- ggplot(summary_speed_data, aes(x = Algorithm, y = Mean_Steps, fill = Algorithm)) +
  geom_col(fill = "#56B4E9", width = 0.65, color = NA) +
  
  geom_errorbar(
    aes(ymin = CI_Lower, ymax = CI_Upper),
    width = 0.15,
    color = "#2C3E50", 
    linewidth = 0.6
  ) +
  
  # Structural label showing the environment scale context
  facet_wrap(~Map) +
  scale_y_continuous(expand = expansion(mult = c(0, 0.15))) +
  
  labs(
    x = "Control Algorithm",
    y = "Mean Steps to Convergence (with 95% CI)"
  ) +
  
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text        = element_text(family = "Arial", size = 12),
    axis.title  = element_text(face = "bold", size = 12),
    axis.text   = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 45, hjust = 1), 
    
    strip.background = element_blank(), 
    strip.text       = element_blank(),
    panel.spacing    = unit(1.5, "lines")
  )

print(speed_plot)

# Save high-resolution figure
ggsave(
  filename = "4.5.1_Expanded_HMap_Convergence_Speed.png", 
  plot     = speed_plot, 
  device   = "png", 
  width    = 6, 
  height   = 5.5, 
  units    = "in",
  dpi      = 600
)

# ─────────────────────────────────────────────────────────────────────────────
# STEP 6: STATISTICAL INFERENCE (LINEAR MODEL & POST-HOC TUKEY)
# ─────────────────────────────────────────────────────────────────────────────
cat("\n======================================================\n")
cat("RUNNING LINEAR CONVERGENCE MODEL: EXPANDED H-MAP")
cat("\n======================================================\n")

# Fit the linear regression model for convergence step costs
speed_model_expanded <- lm(Total_Steps ~ Algorithm, data = raw_speed_data_filtered)

# Extract post-hoc pairwise speed comparisons with Tukey adjustments
speed_comparisons_expanded <- emmeans(speed_model_expanded, pairwise ~ Algorithm, adjust = "tukey")

cat("\n--- Estimated Mean Operational Steps (True Success Only) ---\n")
print(speed_comparisons_expanded$emmeans)

cat("\n--- Pairwise Contrast Matrices (Tukey-Adjusted p-values) ---\n")
print(speed_comparisons_expanded$contrasts)



