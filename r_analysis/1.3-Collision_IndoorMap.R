library(ggplot2)
library(tidyr)
library(dplyr)
library(extrafont)
library(emmeans)
library(brglm2)

# --- STEP 1: INITIALIZE FONTS (For Times New Roman 12pt) ---
# Run font_import(pattern = "Times", prompt = FALSE) once if you haven't before
loadfonts(device = "pdf", quiet = TRUE)
# --- STEP 2: ASSIGN EXISTING DATA FRAMES ---
# Mapping your pre-imported environment objects
DATA_DIR <- "../data" 

path_IndoorMap_1s <- file.path(DATA_DIR, "my_map/IndoorMap_1s.xlsx")
path_IndoorMap_5s <- file.path(DATA_DIR, "my_map/IndoorMap_5s.xlsx")

df_IndoorMap_1s_raw <- read_xlsx(path_IndoorMap_1s)
df_IndoorMap_5s_raw <- read_xlsx(path_IndoorMap_5s)

# Dynamically force the first column name to "Metric" regardless of its current name
colnames(df_IndoorMap_1s_raw)[1] <- "Metric"
colnames(df_IndoorMap_5s_raw)[1] <- "Metric"


# --- STEP 3: TRANSFORM WIDE TO LONG FORMAT ---
IndoorMap_process_dataset <- function(df, duration_label) {
  df %>%
    # Clean string spacing/capitalization discrepancies if they exist
    mutate(Metric = trimws(tolower(Metric))) %>%
    
    # Filter out only the 5 Metric categories needed for the stacked bars
    # (Adjust strings here if your rows use slightly different naming)
    filter(Metric %in% c("true success", "false success", "collision", "timeout", "other failure")) %>%
    
    # Pivot the 7 algorithm columns into a single long column
    pivot_longer(cols = -Metric, names_to = "Algorithm", values_to = "Count") %>%
    
    # Label the duration group
    mutate(Duration = duration_label)
}

# Run the transformation pipeline
cleaned_IndoorMap_1s <- IndoorMap_process_dataset(df_IndoorMap_1s_raw, "1s Duration")
cleaned_IndoorMap_5s <- IndoorMap_process_dataset(df_IndoorMap_5s_raw, "5s Duration")
plot_data_IndoorMap  <- bind_rows(cleaned_IndoorMap_1s, cleaned_IndoorMap_5s)

# --- STEP 4: STANDARDIZE LABELS & LAYER ORDER (UPDATED) ---
# Capture the exact column order of your algorithms from the original dataset
# (We use [-1] to skip the first column, which is the "Metric" column)
original_algo_order_IndoorMap <- colnames(df_IndoorMap_1s_raw)[-1]

plot_data_IndoorMap <- plot_data_IndoorMap %>%
  mutate(
    Metric = case_when(
      Metric == "true success"   ~ "True Success",
      Metric == "false success"  ~ "False Success",
      Metric == "collision"      ~ "Collision",
      Metric == "timeout"        ~ "Timeout",
      Metric == "other failure"  ~ "Other Failure",
      TRUE                       ~ Metric
    ),
    # 1. Enforce the exact dataset column order for the X-axis
    Algorithm = factor(Algorithm, levels = original_algo_order_IndoorMap),
    
    # 2. Sets the visual stack ordering from bottom to top
    Metric = factor(Metric, levels = c("Other Failure", "Timeout", "Collision", "False Success", "True Success"))
  )


# ==========================================
# ==========================================
# ==========================================
# ==========================================

# Collision Analysis
# ==========================================

# 1. Filter out Hard Coded Safety Safety Constraint (D-Opt and Constrained Random)  runs to level the playing field
df_coll_indoor <- subset(plot_data_IndoorMap, 
                        Metric %in% c("True Success", "False Success", "Collision", "Timeout") & 
                          Algorithm %in% c("Standard AIC", 
                                           "Deep AIC", 
                                           "Full-Distribution AIC",
                                           "Purely Epistemic AIC", 
                                           "Unconstrained Random"))


# Step 2: Code the binary dependent variable
df_coll_indoor$Is_Collision <- ifelse(df_coll_indoor$Metric == "Collision", 1, 0)

# --- INDOOR 1s DURATION ---
df_indoor_1s <- subset(df_coll_indoor, Duration == "1s Duration")

model_indoor_1s <- glm(Is_Collision ~ Algorithm, 
                       data = df_indoor_1s, 
                       weights = Count, 
                       family = binomial,
                       method = "brglmFit") # <- Handles 0 counts cleanly

pairs_indoor_1s <- emmeans(model_indoor_1s, pairwise ~ Algorithm, type = "response")
print(pairs_indoor_1s$emmeans)
print(pairs_indoor_1s$contrasts)

# --- INDOOR 5s DURATION ---
df_indoor_5s <- subset(df_coll_indoor, Duration == "5s Duration")

model_indoor_5s <- glm(Is_Collision ~ Algorithm, 
                       data = df_indoor_5s, 
                       weights = Count, 
                       family = binomial,
                       method = "brglmFit") # <- Handles 0 counts cleanly

pairs_indoor_5s <- emmeans(model_indoor_5s, pairwise ~ Algorithm, type = "response")
print(pairs_indoor_5s$emmeans)
print(pairs_indoor_5s$contrasts)

