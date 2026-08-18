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


# 1. Filter out collision runs to level the playing field
df_discounted <- subset(plot_data_IndoorMap, 
                        Metric %in% c("True Success", "False Success", "Timeout") & 
                          Algorithm %in% c("Standard AIC", 
                                           "Deep AIC", 
                                           "Full-Distribution AIC",
                                           "Purely Epistemic AIC", 
                                           "Constrained Random",
                                           "D-Optimality"))

# ==========================================
# ==========================================
# ==========================================
# ==========================================

# Timeout Analysis
# ==========================================


library(emmeans)
library(brglm2) 

# Step 1: Filter metrics and select your 5 algorithms
df_eff_indoor <- subset(df_discounted, 
                        Metric %in% c("True Success", "False Success", "Timeout") & 
                          Algorithm %in% c("Standard AIC", "Deep AIC", "Full-Distribution AIC", "Purely Epistemic AIC", "Constrained Random", "D-Optimality"))

# Step 2: Code the binary dependent variable
df_eff_indoor$Is_Timeout <- ifelse(df_eff_indoor$Metric == "Timeout", 1, 0)

# --- INDOOR 1s DURATION ---
df_indoor_1s <- subset(df_eff_indoor, Duration == "1s Duration" | Duration == "1s")

model_indoor_1s <- glm(Is_Timeout ~ Algorithm, 
                       data = df_indoor_1s, 
                       weights = Count, 
                       family = binomial,
                       method = "brglmFit") # <- Handles 0 counts cleanly

pairs_indoor_1s <- emmeans(model_indoor_1s, pairwise ~ Algorithm, type = "response")
print(pairs_indoor_1s$emmeans)
print(pairs_indoor_1s$contrasts)

# --- INDOOR 5s DURATION ---
df_indoor_5s <- subset(df_eff_indoor, Duration == "5s Duration" | Duration == "5s")

model_indoor_5s <- glm(Is_Timeout ~ Algorithm, 
                       data = df_indoor_5s, 
                       weights = Count, 
                       family = binomial,
                       method = "brglmFit") # <- Handles 0 counts cleanly

pairs_indoor_5s <- emmeans(model_indoor_5s, pairwise ~ Algorithm, type = "response")
print(pairs_indoor_5s$emmeans)
print(pairs_indoor_5s$contrasts)

# ==========================================
# ==========================================
# ==========================================
# ==========================================
# ==========================================
# Precision Analysis
# ==========================================



# 2. Precision Model: Within converged runs, what are the odds of True Success?
df_converged_success <- subset(df_discounted, Metric %in% c("True Success", "False Success"))
df_converged_success$Is_True_Success <- ifelse(df_converged_success$Metric == "True Success", 1, 0)

# Step 1: Subset the data for the 1s condition
# Note: Adjust "1s Duration" if your string looks exactly like "1s" or "1s Duration"
df_1s <- subset(df_converged_success, Duration == "1s Duration" | Duration == "1s")

# Step 2: Run the Weighted Binomial GLM
model_1s <- glm(Is_True_Success ~ Algorithm, 
                data = df_1s, 
                weights = Count, 
                family = binomial,
                method = "brglmFit")

# Step 3: Extract the full pairwise comparisons matrix
pairs_1s <- emmeans(model_1s, pairwise ~ Algorithm, type = "response")

# View the results
pairs_1s$emmeans   # Shows predicted success probabilities per algorithm
pairs_1s$contrasts # Shows the pairwise comparisons (Odds Ratios and p-values)

# Step 1: Subset the data for the 5s condition
df_5s <- subset(df_converged_success, Duration == "5s Duration" | Duration == "5s")

# Step 2: Run the Weighted Binomial GLM
model_5s <- glm(Is_True_Success ~ Algorithm, 
                data = df_5s, 
                weights = Count, 
                family = binomial,
                method = "brglmFit")

# Step 3: Extract the full pairwise comparisons matrix
pairs_5s <- emmeans(model_5s, pairwise ~ Algorithm, type = "response")

# View the results
pairs_5s$emmeans
pairs_5s$contrasts

