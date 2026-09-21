# "json" and "mdns" are deliberately absent from REQUIRES: both are managed
# components declared in idf_component.yml, and the component manager adds them
# as requirements itself. Listing "json" here is what fails on ESP-IDF 6.0 with
# "Failed to resolve component 'json'" - the built-in component was removed.
idf_component_register(
    SRCS "web_server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif esp_http_server robot_state motor
    EMBED_FILES "www/index.html"
)
