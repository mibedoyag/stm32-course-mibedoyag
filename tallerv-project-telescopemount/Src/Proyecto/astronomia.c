/**
 * @file    : astronomia.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de los cálculos de trigonometría esférica.
 */
#include "Proyecto/astronomia.h"

/* Inicialización estricta de estructuras a 0 */
Coordenadas_t target_actual = {0.0f, 0.0f, 0.0f, 0.0f};
Coordenadas_t offset_calibracion = {0.0f, 0.0f, 0.0f, 0.0f};

DatosGPS_t gps_actual = {0.0f, 0.0f, 0, 0, 0, 0.0f};

/* Catálogo Messier (M1 - M20) */
const ObjetoCeleste_t catalogo_messier[] = {
    {"M1 Crab Neb.", 5.57f, 22.01f}, {"M31 And. Gal.", 0.71f, 41.26f},
    {"M42 Orion Neb.", 5.35f, -5.39f}, {"M45 Pleiades", 3.78f, 24.11f},
    {"M13 Herc. Clus.", 16.69f, 36.46f}, {"M57 Ring Neb.", 18.89f, 33.03f},
    {"M8 Lagoon Neb.", 18.06f, -24.37f}, {"M11 Wild Duck", 18.85f, -6.26f},
    {"M27 Dumbell", 19.99f, 22.72f}, {"M33 Triang. G.", 1.57f, 30.66f},
    {"M51 Whirlpool", 13.50f, 47.19f}, {"M81 Bode Gal.", 9.92f, 69.06f},
    {"M82 Cigar Gal.", 9.93f, 69.68f}, {"M92 Glob. Cl.", 17.28f, 43.13f},
    {"M3 Glob. Cl.", 13.71f, 28.38f}, {"M104 Sombrero", 12.66f, -11.62f},
    {"M6 Butterfly", 17.68f, -32.20f}, {"M7 Ptol. Clus.", 17.90f, -34.80f},
    {"M15 Glob. Cl.", 21.50f, 12.16f}, {"M20 Trifid", 18.03f, -23.01f}
};

/* Catálogo de Estrellas Principales */
const ObjetoCeleste_t catalogo_estrellas[] = {
    {"Sirius", 6.75f, -16.71f}, {"Vega", 18.61f, 38.78f},
    {"Betelgeuse", 5.91f, 7.40f}, {"Rigel", 5.25f, -8.20f},
    {"Antares", 16.49f, -26.43f}, {"Aldebaran", 4.59f, 16.50f},
    {"Pollux", 7.75f, 28.01f}, {"Spica", 13.41f, -11.16f},
    {"Arcturus", 14.25f, 19.18f}, {"Deneb", 20.69f, 45.28f}
};

/**
 * @brief Retorna el puntero al objeto solicitado en el catálogo.
 * @param categoria 0: Messier, 1: Estrellas
 * @param indice Índice dentro del catálogo
 */
const ObjetoCeleste_t* Astronomia_ObtenerObjeto(uint8_t categoria, uint8_t indice) {
    if (categoria == 0) return &catalogo_messier[indice % 20];
    return &catalogo_estrellas[indice % 10];
}


/**
 * @brief Inicializa a cero las variables del motor astronómico.
 */
void Astronomia_InitLogica(void) {
    // Reset de coordenadas objetivo
    target_actual.ra = 0.0f;
    target_actual.dec = 0.0f;
    target_actual.altitud = 0.0f;
    target_actual.azimut = 0.0f;

    // Reset de offsets del lazo cerrado
    offset_calibracion.ra = 0.0f; // No usado para offset, pero se inicializa
    offset_calibracion.dec = 0.0f;
    offset_calibracion.altitud = 0.0f;
    offset_calibracion.azimut = 0.0f;

    // Reset de datos GPS
    gps_actual.latitud = 0.0f;
    gps_actual.longitud = 0.0f;
    gps_actual.anio = 0;
    gps_actual.mes = 0;
    gps_actual.dia = 0;
    gps_actual.ut_horas = 0.0f;
}

/**
 * @brief Calcula la Altitud y el Azimut a partir de la RA y Dec actuales.
 * Utiliza la fecha, hora y ubicación del struct gps_actual.
 */
void Astronomia_CalcularAltAz(void) {
    /* * 1. Cálculo del Día Juliano (Aproximación válida para años 2000-2099)
     * Basado en algoritmos astronómicos estándar.
     */
    float d = 367.0f * gps_actual.anio - floorf((7.0f * (gps_actual.anio + floorf((gps_actual.mes + 9.0f) / 12.0f))) / 4.0f)
              + floorf((275.0f * gps_actual.mes) / 9.0f) + gps_actual.dia - 730530.0f;
    d = d + (gps_actual.ut_horas / 24.0f);

    /* 2. Cálculo del Tiempo Sideral Local (LST) en grados */
    float lst_grados = 280.46061837f + 360.98564736629f * d + gps_actual.longitud;
    // Normalizar LST a 0-360 grados
    lst_grados = fmodf(lst_grados, 360.0f);
    if (lst_grados < 0.0f) {
        lst_grados += 360.0f;
    }

    /* 3. Cálculo del Ángulo Horario (HA) en grados */
    // La RA viene en horas decimales (0-24), la convertimos a grados (* 15)
    float ra_grados = target_actual.ra * 15.0f;
    float ha_grados = lst_grados - ra_grados;

    /* Conversión a Radianes para la FPU de C */
    float ha_rad = ha_grados * DEG2RAD_F;
    float dec_rad = target_actual.dec * DEG2RAD_F;
    float lat_rad = gps_actual.latitud * DEG2RAD_F;

    /* 4. Trigonometría Esférica: Cálculo de la Altitud */
    // sin(Alt) = sin(Dec)*sin(Lat) + cos(Dec)*cos(Lat)*cos(HA)
    float sin_alt = (sinf(dec_rad) * sinf(lat_rad)) + (cosf(dec_rad) * cosf(lat_rad) * cosf(ha_rad));
    float alt_rad = asinf(sin_alt);

    /* 5. Trigonometría Esférica: Cálculo del Azimut */
    // cos(Az) = (sin(Dec) - sin(Alt)*sin(Lat)) / (cos(Alt)*cos(Lat))
    float cos_az = (sinf(dec_rad) - (sinf(alt_rad) * sinf(lat_rad))) / (cosf(alt_rad) * cosf(lat_rad));

    // Evitar errores de punto flotante fuera del dominio de acosf [-1, 1]
    if (cos_az > 1.0f) cos_az = 1.0f;
    if (cos_az < -1.0f) cos_az = -1.0f;

    float az_rad = acosf(cos_az);

    // Ajuste de cuadrante para el Azimut según el Ángulo Horario
    float sin_ha = sinf(ha_rad);
    if (sin_ha > 0.0f) {
        az_rad = (2.0f * PI_F) - az_rad;
    }

    /* 6. Convertir de nuevo a grados y aplicar el offset del SYNC */
    target_actual.altitud = (alt_rad * RAD2DEG_F) + offset_calibracion.altitud;
    target_actual.azimut = (az_rad * RAD2DEG_F) + offset_calibracion.azimut;

    // Normalizar Azimut (0 a 360) después del offset
    target_actual.azimut = fmodf(target_actual.azimut, 360.0f);
    if (target_actual.azimut < 0.0f) {
        target_actual.azimut += 360.0f;
    }
}

/**
 * @brief Función para el botón SYNC.
 * Calcula la diferencia (error) entre dónde cree la matemática que está el tubo
 * y dónde están reportando los encoders físicos que realmente está.
 * @param encoder_alt_actual Lectura en grados del encoder de Altitud.
 * @param encoder_az_actual Lectura en grados del encoder de Azimut.
 */
void Astronomia_SyncOffset(float encoder_alt_actual, float encoder_az_actual) {
    // Si el usuario centra el objeto, el encoder físico dice la verdad.
    // Restamos el valor temporal de la altitud calculada para sacar la diferencia geométrica.

    // Calculamos temporalmente la altitud/azimut puros (sin el offset previo)
    float altitud_pura = target_actual.altitud - offset_calibracion.altitud;
    float azimut_puro = target_actual.azimut - offset_calibracion.azimut;

    // El nuevo offset es la diferencia entre el encoder real y la matemática pura
    offset_calibracion.altitud = encoder_alt_actual - altitud_pura;
    offset_calibracion.azimut = encoder_az_actual - azimut_puro;

    // Al ejecutar nuevamente CalcularAltAz(), el objeto quedará perfectamente centrado.
}
